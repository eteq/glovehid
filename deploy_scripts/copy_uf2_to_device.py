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

import mcpgpio
try:
    gpio_dev = mcpgpio.init()
    mcpgpio.gpios_input(gpio_dev)
except:
    gpio_dev = None
def gpio_toggle_reset(waittime=0.1):
    mcpgpio.gpios_output(gpio_dev, 0)
    time.sleep(waittime/2)
    mcpgpio.gpios_input(gpio_dev)
    time.sleep(waittime/2)


def main(mountpoint=None, busid=None, sudo=False, do_unmount=False,
         attach_timeout=15, reset=False, gpio_bootloader=False):
    sudomaybe = 'sudo ' if sudo else ''

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

                print (f'Waiting up to {attach_timeout} seconds for local machine to catch up to attach')

                # Now we run usbip port until its output changes from before the attach.
                # This may hang for a few seconds while usbipd does its magic, but if it's too long we may be stuck.
                port_output = pre_port_output
                t0 = time.time()
                while port_output == pre_port_output:
                    dt = time.time() - t0
                    if dt > attach_timeout:
                        raise TimeoutExpired(f"usbip port never updated despite waiting {attach_timeout}")
                    port_output = run('usbip port'.split(' '), capture_output=True,
                                    check=True,  text=True,
                                    timeout=attach_timeout - dt).stdout

            remote_busid_to_pidvid = parse_usbip_port(port_output)

            usbdrivename = find_drivedevname_from_pidvid(remote_busid_to_pidvid[busid])

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
            copy(uf2path, mountpoint)
            print(f'Copied {uf2path} to the board!')
            if reset:
                time.sleep(reset)
                gpio_toggle_reset()
                if reattach_after_reset:
                    time.sleep(reattach_after_reset)
                    run(f'usbipd.exe wsl attach -b {busid}', shell=True, check=True)
    finally:
        if mounted and do_unmount:
            check_call(sudomaybe + f'umount {mounted}', shell=True)

        if created_mountpoint:
            try:
                mountpoint.rmdir()
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
        raise IOError(f'no sd* drive matches the ids {pidvid}')
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
    parser.add_argument('-w', '--attach-timeout', type=float, default=15)
    parser.add_argument('-g', '--gpio-bootloader', type=float, default=0, description='the amount of time to wait after double-resetting to let bootloader boot, or 0 to skip')
    parser.add_argument('-r', '--reset', type=float, default=0, description='the amount of time to wait before a reset occurs at the end, or 0 to skip')
    parser.add_argument('--reattach-after-reset', type=float, default=0, description='the amount of time to wait before trying to re-attach the busnumber after reset, or 0 to skip')

    args = parser.parse_args()

    if args.reattach_after_reset and not args.reset:
        print("cannot set --reattach-after-reset if not resetting")
        sys.exit(1)

    try:
        main(mountpoint=args.mountpoint, busid=args.busid,
             sudo=args.sudo, do_unmount=not args.no_unmount,
             attach_timeout=args.attach_timeout, reset=args.reset,
             gpio_bootloader=args.gpio_bootloader,
             reattach_after_reset=args.reattach_after_reset)
    except Exception as e:
        print(f"error: {e}", file=sys.stderr)
        sys.exit(1)