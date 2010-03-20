#!/usr/bin/perl -W -T

use strict;
use Device::SerialPort;
use Time::HiRes qw(sleep);

my $separator = "\n";  # 0x0A
my $start_byte = "\n"; # 0x0A
my $stop_byte = "\r";  # 0x0D
my $image_id = "P6";   # ID for .ppm image
my $color_scaler = 8;

my $port = "/dev/ttyUSB0"; # <-- find right port !
my $link = Device::SerialPort->new($port) || die("could not open port: $port - $!"); # <-- match to right com port

$link->databits(8);
$link->baudrate(57600); # <-- match to arduino settings
$link->parity("none");
$link->stopbits(1);
$| = 1; # buffers disabled

open(FILE, "< $ARGV[0]"); # open .ppm image
  my @image = <FILE>; # load the data in an array
close(FILE);

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

print "x_res: ",$x_res,"\n";
print "y_res: ",$y_res,"\n";
print "c_max: ",$c_max,"\n";

@image = split('',$image[0]); # put every single character into a new line --> access it c-style

my $row;
my $led;

for ($row = 0; $row < 8; $row++) {
  $link->write($start_byte);
  for ($led = 0; $led < 8; $led++) {
    $link->write(pack("C",$row));
    $link->write(pack("C",$led));
    $link->write(pack("C",unpack("C",$image[$row*3*8+$led*3+0])/$color_scaler));
    $link->write(pack("C",unpack("C",$image[$row*3*8+$led*3+1])/$color_scaler));
    $link->write(pack("C",unpack("C",$image[$row*3*8+$led*3+2])/$color_scaler));
    #print $row,$led,"\n";
    #print unpack("C",$image[$row*3*8+$led*3+0]),"\n";
    #print unpack("C",$image[$row*3*8+$led*3+1]),"\n";
    #print unpack("C",$image[$row*3*8+$led*3+2]),"\n\n";
    sleep(0.002);
  }
  $link->write($stop_byte);
}

