import hid

MCP_VID = 0x04d8
MCP_PID = 0x00dd

def write_command(dev, bs):
    allbytes = bs + (64-len(bs))*[0]
    return dev.write(allbytes)


def write_then_read(dev, bs):
    write_command(dev, bs)
    return dev.read(64)


def print_response(bs, upto=64):
    for i, b in enumerate(bs):
        if i > upto:
            return
        print(i, b, hex(b), '0b{:08b}'.format(b))

def gpios_input(dev):
    res = write_then_read(dev, [0x50,0,  0,0,1,1, 0,0,1,1, 0,0,1,1, 0,0,1,1])
    assert res[1] == 0

    res = write_then_read(dev, [0x51])
    return res[2], res[4], res[6], res[8]

def gpios_output(dev, gp0=None, gp1=None, gp2=None, gp3=None):
    # None for input
    towrite = [0x50, 0]
    for gp in [gp0, gp1, gp2, gp3]:
        if gp is None:
            towrite += [0,0,1,1]
        else:
            towrite += [1, gp, 1, 0]

    res = write_then_read(dev,  towrite)
    assert res[1] == 0

def init():
    d = hid.device()
    d.open(MCP_VID, MCP_PID)
    d.set_nonblocking(1)
    return d
