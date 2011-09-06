#!/bin/sh

port=/dev/cu.usbserial-A900a4PQ

# gliders are 

# 1110 -> 14
# 1000 -> 8
# 0100 -> 4

# 11100 -> 28
# 00100 -> 4
# 01000 -> 8

# put us in the mode

./send-cmd.pl $port 2 # attention-both
./send-cmd.pl $port 67 # hold single mode, so we can see our construction
./send-cmd.pl $port 1 # attention-right
./send-cmd.pl $port 70 # clear


#./send-cmd.pl $port 1 # attention-right
#./send-cmd.pl $port 72 # set cursor y
#./send-cmd.pl $port 128 # line 0
#./send-cmd.pl $port 71 # set cursor x
#./send-cmd.pl $port 128 # column 0


echo right is clear\?

read a

# first make a row going up-left

for r in 1 2 3 4 5 6 7 8
do
  ./send-cmd.pl $port 14
done


for r in 1 2 3 4 5 6 7 8
do
  ./send-cmd.pl $port 8
done
for r in 1 2 3 4 5 6 7 8
do
  ./send-cmd.pl $port 4
done

# move down to row 10

./send-cmd.pl $port 0 # attention-left
./send-cmd.pl $port 70 # clear

echo left is clear\?
read a

./send-cmd.pl $port 0 # attention-left
./send-cmd.pl $port 72 # set cursor y
./send-cmd.pl $port 138 # jump to line 10

echo left is still clear\?
read a

#./send-cmd.pl $port 0 # attention-left
#./send-cmd.pl $port 71 # set cursor x
#./send-cmd.pl $port 128 # column 0

# now make a row going up-right

for r in 1 2 3 4 5 6 7 8
do
   ./send-cmd.pl $port 28
done
for r in 1 2 3 4 5 6 7 8
do
   ./send-cmd.pl $port 4
done
for r in 1 2 3 4 5 6 7 8
do
   ./send-cmd.pl $port 8
done

echo is it ok - if not hit ctrl c
read a

# if everything's alright here, we tell life to start going again
./send-cmd.pl $port 2 # attention-all
./send-cmd.pl $port 65 # explicit life
