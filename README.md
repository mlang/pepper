# Synopsis

Pepper is a Bela/Salt project to create a Braille display enabled eurorack
multi-function module.

# Installation

Before you can build this project on the Bela platform,
you need to install the following additional packages on the target system:

```shell
Bela:~# apt install brltty libbrlapi-dev libboost-dev libboost-serialization-dev lv2-dev
```

The recommended way to manage the project source is to check it out directly
via Git on the target system:

```shell
Bela:~# cd /root/Bela/projects
Bela:~# git clone https://github.com/mlang/pepper
```

# Building

You can now build the project with:

```shell
Bela:~# make -C/root/Belai/projects/pepper
```

And you can run pepper with:

```shell
Bela:~# make -C/root/Bela run PROJECT=pepper
```

# Modes

As of now, a very basic sequencer, a crude level meter,
and two software oscillator modes (using LV2 plugins) are implemented.

# Usage

You need to connect a USB capable Braille display to your Salt programmable
eurorack module before booting it up.  When pepper starts, it
will show the current mode name.  Scrolling downwards
reveals the specific information for that particular mode.
Scrolling to the right/left while the mode name is being displayed
will change the current mode.  You can imagine the overall user interface
like a browser with different tabs.  To switch to a different
tab, scroll to the topmost line and then to the left or right as desired.

## Sequencer

The Sequencer has 4 trigger and 4 CV+Gate channels.
While the trigger channels are just implemented as a grid of braille cells,
the CV channels are implemented like a piano roll.
Using the cursor routing keys you can set new trigger or CV values.

