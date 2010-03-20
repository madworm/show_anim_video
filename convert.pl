#!/usr/bin/perl -W -T

use strict;

my $separator = "\n";  # 0x0A
my $image_id = "P6";   # ID for .ppm image
my $color_scaler = 8;

$| = 1; # buffers disabled

open(FILE, "< $ARGV[0]"); # open .ppm image
  my @image = <FILE>; # load the data in an array
close(FILE);

my $frame_number = $ARGV[0];
$frame_number =~ m/^.*(\d{1,3})\..*$/;
$frame_number = $1;

my $tmp = $image[0]; # get .ppm ID from image
$tmp =~ s/\n//;      # remove \n

shift @image; #remove ID line

if ( $tmp ne $image_id ) { print "\nnot .ppm image!\n\n"; exit; }

$tmp = $image[0];
if ( $tmp =~ m/^#.*/ ) { # comment line ?
  shift @image; # remove comment line
}

my $x_res = $image[0];
$x_res =~ m/^(\d*)\ /;
$x_res = $1;
my $y_res = $image[0];
$y_res =~ m/^\d*\ (\d*)/;
$y_res = $1;
my $c_max = $image[1];

shift @image; # remove X x Y line
shift @image; # remove color line

if ( ($x_res > 8) || ($y_res > 8) ) { print "\n($x_res x $y_res)\nimage larger than 8x8\n\n"; exit; }

@image = split('',$image[0]); # put every single character into a new line --> access it c-style

my $row;
my $led;
my $string = "";

$string .= "const char PROGMEM frame$frame_number\[193\] \= \{";

for ($row = 0; $row < 8; $row++) {
  for ($led = 0; $led < 8; $led++) {
    $string .= int(unpack("C",$image[$row*3*8+$led*3+0])/$color_scaler);
    $string .= ",";
    $string .= int(unpack("C",$image[$row*3*8+$led*3+1])/$color_scaler);
    $string .= ",";
    $string .= int(unpack("C",$image[$row*3*8+$led*3+2])/$color_scaler);
    $string .= ",";
  }
}

$string =~ s/\,$/\}\;/; # replace last , with };

print "\n\n";
print $string;
print "\n\n";
