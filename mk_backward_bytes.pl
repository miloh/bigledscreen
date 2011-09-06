#!/usr/bin/perl -w

open(OUT, ">backward_bytes.h") || die "backward_bytes.h: $!";

for (my $b = 0; $b <= 255; $b++) {
    print OUT "[0b";
    for (my $bit = 7; $bit >= 0; $bit--) {
	if ($b & (1 << $bit)) {
	    print OUT "1";
	} else {
	    print OUT "0";
	}
    }
    print OUT "u] = 0b";
    for (my $bit = 0; $bit <= 7; $bit++) {
	if ($b & (1 << $bit)) {
	    print OUT "1";
	} else {
	    print OUT "0";
	}
    }
    print OUT "u,\n";
}
close(OUT);
