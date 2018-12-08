# Deep Blue - Hobby OS Targeting Intel x86 Computers

Copyright (C) 2016-2018 Don Clarke -- see LICENSE.TXT

This repository contains various boot-loaders along with the kernel for the Deep Blue OS.

## Boot Loaders

### Floppy Boot Sectors

fbs_raw.asm     : The boot sector will load the rest of the boot loader and the kernel from a raw (unformatted) floppy disk.

fbs_fat12.asm   : The boot sector will load the stage 2 boot loader from the root directory of the FAT12 formatted floppy disk.

### Hard Drive Boot Records

## Deep Blue OS Kernel

## Building

build.bat   : Windows build batch file.
