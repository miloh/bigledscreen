#!/usr/bin/perl -w
use Device::SerialPort;
use IO::File;

my $device=shift @ARGV;

my $port = new Device::SerialPort($device);
$port->baudrate(19200);

#$port->write(chr(255).chr(255));
#$port->write(chr(0b10101010).chr(255).chr(254).chr(253).chr(252).chr(251).chr(250).chr(0b01010101));
$port->write(chr($ARGV[0]));
sleep(2);
#(undef, $string_in) = $port->read(1);
#print ord($string_in)."\n";
#(undef, $string_in) = $port->read(1);
#print ord($string_in)."\n";
#sleep(1);
#$port->write(chr(0b10101010));
#$port->write(chr(2)); # address both
#$port->flush();
#sleep(1);
#$port->write(chr(255));
#$port->write(chr(66)); # set test pattern state
#$port->write(chr(255));
#$port->flush();
# print $port chr(64); # set random state
