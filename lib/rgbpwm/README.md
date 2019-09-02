# RGBLeds Application


```
nrfjprog -f NRF52 -e;nrfjprog -f NRF52 --reset --program rgbpwm_fullmem.hex
```

Open uart with 115200-8n1. Commands available include:  

## Change local colour

```
rgb set 0xWWRRGGBB [delay ms]

ex:
rgb set 0x220022 1000
rgb set 0xaaaaaa 2000
```

## Change remote colour(s)

- **address** is a 16-bit address expressed in hex. 0xffff - all nodes including local node

```
rgb tx <address> 0xWWRRGGBB [delay ms]

ex:
rgb tx 0xffff 0x220022 1000
rgb tx 0x0123 0xaaaaaa 2000
```

# Configuration

The app stores settings in flash to retain them between power cycles.
The available settings can be shown with the ```config dump``` command:

```
051598 compat> config dump
052598 panmstr/pan_id = 0xDECA
052598 uwb/channel = 5
...
052601 uwb/role = 0x0000                                 # This board's role in the network, master = 0x7, all others 0x0
052601 split/status = 0
052601 rgbpwm/pwm_freq = 1000                            # Frequency of the LED-PWM
052601 rgbpwm/verbose = 0x0                              # Set to 0xffff to show verbose debug data
052601 rgbpwm/mode = 0x0                                 # Mode 0 each node chooses a random color, 1 all nodes go to same random colour. Only affects master node
052601 rgbpwm/local_delay = 60000                        # (ms) Delay between automatic local colour changes, should be larger than the master delay
052601 rgbpwm/master_delay = 30000                       # (ms) Delay between automatic master colour changes
052602 rgbpwm/colours0 = #ff0000,#00ff00,#0000ff,#000000 # Set of approved colours
052603 rgbpwm/colours1 =                                 # approved colours continued...
052603 rgbpwm/colours2 =                                 # approved colours continued...
052603 rgbpwm/colours3 =                                 # approved colours continued...
...

```

To change a configuration parameter use the following command flow:

```
config <parameter to change> <new value>
config commit                 # Activate changed value
config save                   # Save in permanent memory (flash)
```

## Setting the role

Only one board can be the Master of the network, the others are slaves.
By default a board is a slave that connects to a local master. A slave
has the uwb/role parameter set to 0x0 whilst the master has it set to 0x7.

To promote a board to a master:

```
config uwb/role 0x7
config save
reset
```

The last command is to restart the board as it's only at boot it will
set itself up for the correct role. 

## Example: To the pwm frequency on the local board:

```
config rgbpwm/pwm_freq 800    # Change the frequency of the pwm to 800 hz
config commit                 # Activate change
config save                   # Save in permanent memory so it's retained after power cycle
```

## Changing the list of approved colours the board shifts between

If not controlled over the network the board automatically chooses a new colour
to change to every ```rgbpwm/local_delay``` milliseconds. It chooses the new colour
from the list of approved colours stored in the ```rgbpwm/colours0``` to ```rgbpwm/colours3```
vectors. The reason there are more than one vector is that only 64 characters can be stored
in each configuration parameter.

Each colour is represented as a html-hex code with some caviates:

- Hexadecimal starting with White, Red, Green, and Blue: ```#WWRRGGBB```
- Each colour starts with ```#```
- Each colour can range from ```00```(0) to ```FF```(255) where ```FF``` is the brightest.
- The white channel may or may not be used on the board itself
- The short ```#RGB``` format is not allowed.
- Not all colours need to be included. For instance ```#000000FF``` (blue) is the same as ```#FF```.
- Lower or upper case allowed: ```#aaBBcc == #AAbbCC```

To store a set of new colours, a max of 32 colours allowed:

```
config rgbpwm/colours0 #010101,#020202,#050505,#FF00FF,#EECC99,#123456,#987654,#ABCDEF
config rgbpwm/colours1 #110101,#120202,#050505,#eF00FF,#3ECC99,#323456,#f87654,#fBCDEF
config commit   # Activate
config save     # Save in permanent memory
config dump     # Show current settings
```

## Updating the configuration globally in the network

If you've set the colours, pwm frequency etc on one node and want all the other nodes
to have the same set of parameters you can broadcast the local nodes parameters:

```
rgb txcfg <address, 0xffff=broadcast>

# Example
006989 compat> rgb txcfg 0xffff
009012 # txcfg: 'pwm_freq' -> '2000'
009012 # txcfg: 'local_delay' -> '35000'
009012 # txcfg: 'colours0' -> '#ff0000,#00ff00,#0000ff,#000000'
009012 # txcfg: 'colours1' -> '#aabbcc,#ddeeff'
009012 # txcfg: 'colours2' -> ''
009012 # txcfg: 'colours3' -> ''
```

## Listing the active nodes in the network

```panm list```

```
# Example
009012 compat> panm list
032764 #idx, addr, role, slot, p,  lease, euid,             flags,          date-added, fw-ver
032764    0, 4c13,    1,    0,  ,       , 013A6102C4B44C13,  1000, 1970-01-01T00:00:00, 0.1.1.10
032764    1, d18d,    1,    1,  , 1553.7, 0402C188CAF2D18D,  1000, 1970-01-01T00:00:09, 0.1.1.9

```

## Upload firmware to local node using usb cable

```
# Setup connection, change ttyUSB0 to what's used locally on your computer
newtmgr conn add usb0 type=serial connstring="dev=/dev/ttyUSB0,baud=115200"

# Upload image
newtmgr -c usb0 image upload <path to .img file>
```

```
# See if upload succeeded
$ newtmgr -c usb0 image list
Images:
 slot=0
    version: 0.1.1.12
    bootable: true
    flags: active confirmed
    hash: 0b63d1d178faa26556789daf76bcc0c475c63411549a0876f49037ef3bbd45b4
 slot=1
    version: 0.1.1.13
    bootable: true
    flags: 
    hash: 03945536f260ae23dff500ed95fdb3585bfd618c04dd6ac3e584ca7bf7d46987
Split status: N/A (0)
```

```
# Test the new image (the hash is taken from the list output)
newtmgr -c usb0 image test 03945536f260ae23dff500ed95fdb3585bfd618c04dd6ac3e584ca7bf7d46987
# Restart into the new image, note it will take up to a minute for the new image to be activated
newtmgr -c usb0 reset
```

```
# If the new image works well, we make it permanent
newtmgr -c usb0 image confirm 03945536f260ae23dff500ed95fdb3585bfd618c04dd6ac3e584ca7bf7d46987
```

Should the update fail, or the new image don't work a power cycle should boot the board back in the
old image. ```newtmgr -c usb0 image list``` will show what image is active. 

## Broadcasting new firmware to the network

After the master node have been updated with new firmware from Apache Mynewtmanager or locally
this image can be broadcast to all the other nodes in the network. Note that a single
broadcast takes about 60s. Multiple consecutive broadcasts may be needed in a spread out
network. They're safe in that only a completed brodcast will be booted to. If a node failes
to receive one of the many packets from the master, that packet can be received during a subsequent
broadcast until the node has a complete copy of the new firmware. Only then will the node
reboot into the new image.

```
# Command
bota txim <address> <image number, 0=current, 1=second slot>
```

```
#Example:
bota txim 0fffff 0
057338 compat> bota txim 0xffff 0
059072 compat> ver: 0.1.1.10
059073 [ts=461507780us, mod=70 level=0] Reading flash at 0, 40 bytes rc=0
059098 [ts=461703080us, mod=70 level=1] ver: 0.1.1.10
059098 [ts=461703080us, mod=70 level=0] Reading flash at 0, 40 bytes rc=0
059123 [ts=461898380us, mod=70 level=1] ver: 0.1.1.10
...
061750 [ts=482421848us, mod=70 level=0] Reading flash at 39E90, 220 bytes rc=0
061752 [ts=482437472us, mod=70 level=0] Reading flash at 39F6C, 148 bytes rc=0
bota: resending end
061754 [ts=482453096us, mod=70 level=0] Reading flash at 39FAC, 84 bytes rc=0
bota: resending end
061786 [ts=482703080us, mod=70 level=0] Reading flash at 39FAC, 84 bytes rc=0
bota: resending end
061818 [ts=482953064us, mod=70 level=0] Reading flash at 39FAC, 84 bytes rc=0
bota: resending end
061850 [ts=483203112us, mod=70 level=0] Reading flash at 39FAC, 84 bytes rc=0
bota: txim finished
```

The above command may have to be run a few times until the ```panm list``` shows the correct
firmware version (fw-ver). If a node still isn't updating try running the ```bota txrst 0xffff``` command
to reset all nodes in the network and then try the ```bota txim 0fffff 0``` command again.

