#!/usr/bin/perl

use Device::SerialPort;
use Expect;
$| = 1;

my %cmds = (
	    "id" => \&show_id,
	    "program-sign-side" => \&program_sign_side,
	    "program-computer-side" => \&program_computer_side,
	    "1-high" => sub { &io("3"); },
	    "3-high" => sub { &io("C"); },
	    "all-low" => sub { &io("0"); },
	    "all-high" => sub { &io("F"); },
	    "monitor" => \& monitor
	    );

sub usage {
    print STDERR "Usage: $0 <serial port> <what to do>\n";
    print STDERR "What to do is one of:\n";
    for my $cmd (keys %cmds) {
	print "  $cmd\n";
    }
    exit(1);
};

if (!@ARGV) {
    &usage;
}

my $device = shift @ARGV;
my $port = new IO::File;
$port->open($device, "w+");

$e = Expect->init($port);
$e->stty(19200);
#$e->debug(1);
$e->log_file(\&logger);

sleep(2);
$e->send("+++");
my @res = $e->expect(5, "OK");
if ($res[1] eq '1:TIMEOUT') {
    print "No response at 19200 baud; Trying 9600 baud\n";
    $e->stty(9600);
    sleep(2);
    $e->send("+++");
    my @res = $e->expect(5, "OK");
    if ($res[1] eq '1:TIMEOUT') {
	print "Could not get xbee to answer\n";
	exit(1);
    }
}

while (my $cmd = shift @ARGV) {
    &{$cmds{$cmd}};
}

sub do_command {
    my ($cmd) = @_;
    my $output = "";
    my $timeouts = 0;

    print "Executing command: $cmd\n";
    $e->send($cmd);

    while ($timeouts < 2) {
	# wait for out to start outputting
	my @pats = $e->expect(1);
	my ($matched_pattern_position, $error, $successfully_matching_string, $before_match, $after_match) = @pats;
	if ($error eq '1:TIMEOUT') {
	    $timeouts++;
	} else {
	    $output .= $before_match;
	}
    }

    return $output;
}


#echos to stdout, translating \r into \n
sub logger {
    my ($logmsg) = @_;
    $logmsg =~ s/\r/\n/g;
    print $logmsg;
};

sub show_id {
    print "Retrieving high part of address\n";
    &do_command("ATSH\r");
    print "Retrieving low part of address\n";
    &do_command("ATSL\r");

    print "Showing local address\r";
    &do_command("ATMY\r");

    print "Retrieving association information\n";
    &do_command("ATAI\r");
    print "Probing DN\n";
    &do_command("ATDN\r");

    print "Probing PAN ID\n";
    &do_command("ATID\r");
    
    # retrieve known nodes
    print "Probing nodes\n";
    &do_command("ATND\r");
    
};

sub program_common {
    # reset to factory defaults so we can start anew
    &do_command("ATRE\r");

    # set baud rate to 19200
    &do_command("ATBD4\r");
    
    # PAN id, common to all xbees that talk:
    &do_command("ATIDe508\r");

    # retry a lot. don't give up.
    &do_command("ATRR6\r");

    # buffer so we can do higher transmit rates
    &do_command("ATRO10\r");
};

sub program_sign_side {
    &program_common();

    # this sign will only talk to something self-identifying as 'b5ef'
    &do_command("ATDLB5EF\r");

    # configure AD0 and AD2 for digital output, default low.  These
    # are connected to a transistor connected to reset pins.
    # when they go high, they cause the transistor to pull down
    # the reset lines.
    &do_command("ATD04\r");
    &do_command("ATD24\r");

    # this sign will allow 'b5ef' to update the output lines:
    &do_command("ATIAB5EF\r");
    
    # do not time out when output lines are changed; leave them
    # in whatever state the remote end specified.  otherwise they
    # return to default values
    &do_command("ATT00\r");
    &do_command("ATT20\r");

    # do not dump messages out serial when any of the digital lines change:
    &do_command("ATIU0\r");

    # write changes to flash
    &do_command("ATAC\r");
    &do_command("ATWR\r");
};

sub program_computer_side {
    &program_common();

    # Identify as the thing the sign will talk to:
    &do_command("ATMYB5EF\r");

    # Configure AD0 and AD2 for digial input:
    &do_command("ATD03\r");
    &do_command("ATD23\r");

    # Configure AD1 and AD3 for digital output, default low.  These are connected
    # to AD0 and AD2, so we can control what goes out the other end.
    &do_command("ATD14\r");
    &do_command("ATD34\r");

    # Send updates whenever AD0 or AD2 are changed:
    &do_command("ATIC5\r");

# ATIR?0x14 for 0&1
# ATIT? 5

    # write new config to flash
    &do_command("ATAC\r");
    &do_command("ATWR\r");
};


sub reset {
    my ($pin, $state) = @_;

    if ($state eq 'high') {
	&do_command("ATD${pin}5\r");
    } elsif ($state eq 'low') {
	&do_command("ATD${pin}4\r");
    } else {
	die "$state must be low or high";
    }
    &do_command("ATAC\r");
    &do_command("ATIS\r");
};

sub io {
    my ($val) = @_;

    &do_command("ATIO$val\r");
#    &do_command("ATIS\r");

    # do not wait for reply; we might want to hurry up and talk to the bootloader
    $e->send("ATCN\r");
    
};

sub monitor {
    $e->expect(3600);
};
