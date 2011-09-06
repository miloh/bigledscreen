port=/dev/cu.usbserial-A900a4PQ

# gliders are 

# 1110 -> 14
# 1000 -> 8
# 0100 -> 4

# 11100 -> 28
# 00100 -> 4
# 01000 -> 8

# 11100 -> 28
# 10000 -> 16
# 01000 -> 8

./send-cmd.pl $port 2 # attention-both
./send-cmd.pl $port 67 # hold single mode, so we can see our construction
echo is held
read a

./send-cmd.pl $port 2 # attention-bot
./send-cmd.pl $port 70 # clear
./send-cmd.pl $port 1 # attention-right
./send-cmd.pl $port 70 # clear

for val in 28 16 8
do
./send-cmd.pl $port 1 # atetntion-right
./send-cmd.pl $port 71 # set cursor x
./send-cmd.pl $port 135 # 7
./send-cmd.pl $port $val
done

./send-cmd.pl $port 2 # attention both
./send-cmd.pl $port 65 explicit life
