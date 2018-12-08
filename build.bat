@echo off

REM This script assumes that 'nasm' is in your path

echo | set /p x=Building fbs_raw...
call nasm fbs_raw.asm -o fbs_raw.flp  -f bin -Wall && (echo Success) || (echo Error!)

echo | set /p x=Building fbs_fat12...
call nasm fbs_fat12.asm -o fbs_fat12.mbr  -f bin -Wall && (echo Success) || (echo Error!)
