#!/usr/bin/env python3

"""
Assumtions of this script:

* You are either 1) in Linux or 2) in a WSL that has usbipd-win installed

If you give a busid, this will first try to attach, then mount in the
mountpoint, then copy.  If you aren't in WSL, instead you can just specify the
mountpoint explicitly.

"""
import sys
import time
from getpass import getuser
from pwd import getpwnam
from pathlib import Path
from shutil import copy
from subprocess import check_call, check_output, run, TimeoutExpired

import make_uf2

from gpio_interface import gpio_toggle_reset, gpio_init

class NoMatchingDriveError(IOError):
    pass

def main(mountpoint=None, busid=None, sudo=False, do_unmount=False,
         attach_timeout=10, reset=False, gpio_bootloader=False,
         reattach_after_reset=0, flashing_timeout=30):
    sudomaybe = 'sudo ' if sudo else ''
    if gpio_bootloader or reset:
        try:
            gpio_init()
        except Exception as e:
            print(f'Failed to initialize GPIO: "{e}". Skipping activities '
                  f'requiring reset pin.')
            gpio_bootloader = reset = False

    if gpio_bootloader:
        gpio_toggle_reset(0.2)
        gpio_toggle_reset()
        time.sleep(gpio_bootloader)

    uf2path = make_uf2.main()
    if mountpoint is None:
        target_dir = uf2path.parent

    isbipdlist_complproc = run('usbipd.exe wsl list', shell=True, capture_output=True, text=True)
    in_wsl = isbipdlist_complproc.returncode == 0

    if mountpoint is None:
        if in_wsl:
            mountpoint = uf2path.parent / "uf2_mount"
        else:
            raise OSError('no mountpoint specified and not in wsl')

    mounted = None
    created_mountpoint = False
    try:
        if in_wsl:
            if busid is None:
                raise ValueError('busid must be given if in wsl')

            for line in isbipdlist_complproc.stdout.split('\n'):
                if line.startswith(busid) and  'Attached -' in line:
                    print(f'Busid {busid} is already attached to wsl.  Skipping.')
                    port_output = run('usbip port'.split(' '), check=True, capture_output=True, text=True).stdout
                    break
            else:
                pre_port_output = run('usbip port'.split(' '), check=True, capture_output=True, text=True).stdout

                print(f'Attaching busid {busid} to wsl')
                attach_compproc = run(f'usbipd.exe wsl attach -b {busid}', shell=True)
                if attach_compproc.returncode != 0:
                    raise ValueError('usbipd could not attach (see above)')

            print(f"Waiting up to {attach_timeout} seconds for drive to appear")
            t0 = time.time()
            while (time.time() - t0) < attach_timeout:
                retproc = run('usbip port'.split(' '), capture_output=True,
                                text=True, timeout=attach_timeout - time.time() + t0)
                if retproc.returncode == 0:
                    remote_busid_to_pidvid = parse_usbip_port(retproc.stdout)
                    try:
                        usbdrivename = find_drivedevname_from_pidvid(remote_busid_to_pidvid[busid])
                        break
                    # KeyError means the busid didn't show up in usbip port
                    except (NoMatchingDriveError, KeyError):
                        pass
                # lets not overwhelm the poort computer, only do the loop above ~20 times at most
                time.sleep(attach_timeout/20)
            else:
                raise NoMatchingDriveError(f'never got a /dev/sd* drive that '
                                           f'is attached to the busid {busid}')

            mountdev = Path(f'/dev/{usbdrivename}')
            if not mountdev.is_block_device():
                raise IOError(f'{mountdev} does not exist or is not a block device')

            if mountpoint.exists():
                if mountpoint.is_dir():
                    print(f'Warning: mountpoint {mountpoint} already exists but we will be mounting into it')
                else:
                    raise IOError(f'Mountpoint {mountpoint} exists and is not a directory')
            else:
                print(f'Making directory {mountpoint} to be a mountpoint')
                mountpoint.mkdir()
                created_mountpoint = True

            print("Mounting", mountdev, 'to', mountpoint)
            user_passwd = getpwnam(getuser())
            mountoptions = f'-o uid={user_passwd.pw_uid},gid={user_passwd.pw_gid}'
            check_call(sudomaybe + f'mount {mountoptions} {mountdev} {mountpoint}', shell=True)
            if mounted is None:
                mounted = mountpoint

        if not mountpoint.is_dir():
            raise IOError(f'Requested mount point {mountpoint} is not a directory')
        else:
            if not list(mountpoint.glob('CURRENT.UF2')):
                raise IOError(f'drive {mountdev} is mountable, but does not look like a UF2 drive')

            print(f'Copying {uf2path} to the board')
            copy(uf2path, mountpoint)

            t0 = time.time()
            while time.time() - t0 < flashing_timeout:
                if not mountdev.is_block_device():
                    print(f"{mountdev} is gone, flashing seems to be complete.")
                    break
            else:
                raise IOError(f'Drive is still there after {flashing_timeout}, did the flash actually succeed?')

            if reset:
                time.sleep(reset)
                print("Resetting board")
                gpio_toggle_reset()
                if reattach_after_reset and in_wsl:
                    print(f"Reattaching {busid}")
                    time.sleep(reattach_after_reset)
                    run(f'usbipd.exe wsl attach -b {busid}', shell=True, check=True)
    finally:
        if mounted and do_unmount:
            print(f"unmounting {mounted}")
            check_call(sudomaybe + f'umount {mounted}', shell=True)

        if created_mountpoint:
            try:
                mountpoint.rmdir()
                print(f"Removed {mountpoint}")
            except:
                pass # We tried 🤷

def parse_usbip_port(usbip_port_output):
    # Example output without sudo:
    # Imported USB devices
    # ====================
    # libusbip: error: fopen
    # libusbip: error: read_record
    # Port 00: <Port in Use> at Full Speed(12Mbps)
    #        Adafruit : unknown product (239a:0051)
    #        1-1 -> unknown host, remote port and remote busid
    #            -> remote bus/dev 008/002
    # libusbip: error: fopen
    # libusbip: error: read_record
    # Port 01: <Port in Use> at High Speed(480Mbps)
    #        Silicon Motion, Inc. - Taiwan (formerly Feiya Technology Corp.) : Flash Drive (090c:1000)
    #        1-2 -> unknown host, remote port and remote busid
    #            -> remote bus/dev 009/001
    remote_bid_to_pidvid = {}

    in_port = None
    vidpid = None

    for line in usbip_port_output.split('\n'):
        if line.startswith('Port'):
            in_port = line.split(':')[0]
            vidpid = None
        elif in_port:
            if vidpid is None:
                # first line, should have the vidpid in it
                vidpid = line.split('(')[-1].split(')')[0].strip()
            elif 'remote bus/dev' in line:
                busdev = line.split()[-1].strip().split('/')
                assert len(busdev) == 2
                busid = '-'.join([str(int(e)) for e in busdev])
                if vidpid is not None:
                    remote_bid_to_pidvid[busid] = vidpid
                    in_port = None
                    vidpid = None


    return remote_bid_to_pidvid

def find_drivedevname_from_pidvid(pidvid):
    vid_tofind, pid_tofind = pidvid.split(':')

    to_return = []

    sysblock = Path('/sys/block')
    for drive in sysblock.glob('sd*'):  # are USB mass storage's always sd*s?
        if drive.is_symlink():
            linkpath = drive.resolve()
            if 'vhci' not in str(linkpath) and 'usb' not in str(linkpath):
                continue
                # this is not a usb drive
                #TODO: decide if this works for "real" usb drives.  Could just drop the if staten
            for parent in linkpath.parents:
                vidpath = parent / 'idVendor'
                pidpath = parent / 'idProduct'
                if vidpath.is_file() and pidpath.is_file():
                    with vidpath.open('r') as f:
                        vid = f.read().strip()
                    with pidpath.open('r') as f:
                        pid = f.read().strip()

                    if pid == pid_tofind and vid == vid_tofind:
                        to_return.append(drive.stem)
    if len(to_return) == 0:
        raise NoMatchingDriveError(f'no sd* drive matches the ids {pidvid}')
    elif len(to_return) > 1:
        raise IOError(f'found multiple drives: {to_return} matching USB device '
                      f'{pidvid} - do not know which one to pick!')
    else:
        return to_return[0]

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('-b', '--busid', default=None)
    parser.add_argument('-m', '--mountpoint', default=None)
    parser.add_argument('-s', '--sudo', action='store_true')
    parser.add_argument('--no-unmount', action='store_true')
    parser.add_argument('-w', '--attach-timeout', type=float, default=10)
    parser.add_argument('-g', '--gpio-bootloader', type=float, default=0, help='the amount of time to wait after double-resetting to let bootloader boot, or 0 to skip')
    parser.add_argument('-r', '--reset', type=float, default=0, help='the amount of time to wait after flashing is completed to reset the board, or 0 to skip.')
    parser.add_argument('--reattach-after-reset', type=float, default=0, help='the amount of time to wait before trying to re-attach the busnumber after reset, or 0 to skip')
    parser.add_argument('-t', '--flashing-timeout', type=float, default=30, help='the maximum amount of time to wait for flashing to complete')
    parser.add_argument('--debug', action='store_true')

    args = parser.parse_args()

    if args.reattach_after_reset and not args.reset:
        print("cannot set --reattach-after-reset if not resetting")
        sys.exit(1)

    try:
        main(mountpoint=args.mountpoint, busid=args.busid,
             sudo=args.sudo, do_unmount=not args.no_unmount,
             attach_timeout=args.attach_timeout, reset=args.reset,
             gpio_bootloader=args.gpio_bootloader,
             reattach_after_reset=args.reattach_after_reset,
             flashing_timeout=args.flashing_timeout)
    except Exception as e:
        if args.debug:
            raise
        else:
            print(f"error: {e}", file=sys.stderr)
            sys.exit(1)