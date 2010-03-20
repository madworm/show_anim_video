#!/usr/bin/perl -W

use strict;
use Device::SerialPort;
use Time::HiRes qw(sleep);

use Imager;
use Imager::Screenshot 'screenshot';

my $start_byte = "\n"; # 0x0A
my $stop_byte = "\r";  # 0x0D
my $color_scaler = 8;  # image has 0-255 for each color, too much for the matrix board

my $port = "/dev/ttyUSB0"; # <-- find right port !
my $link = Device::SerialPort->new($port) || die("could not open port: $port - $!"); # <-- match to right com port

$link->databits(8);
$link->baudrate(57600); # <-- match to arduino settings
$link->parity("none");
$link->stopbits(1);
$| = 1; # buffers disabled
sleep 4; # wait until the arduino bootloader has timed out

my $row;
my $led;
my $red;
my $green;
my $blue;
my $alpha;
my @scanline;
my $img;
my $newimg;
my $line;
my @coords;
my $x;
my $y;
my $left;
my $right;
my $top;
my $bottom;

my $x_box_size = $ARGV[0] || 8;
my $y_box_size = $ARGV[1] || 8;

while (1) {

@coords = `xinput query-state Mouse[1]`; # you need to adapt this to your system. use "xinput list" to get all X11 devices
foreach $line ( @coords ) {
  $line =~ m/^.*valuator\[0\]\=(\d{1,4})/;
  $x = $1;
  $line =~ m/^.*valuator\[1\]\=(\d{1,4})/;
  $y = $1;
}

$left = $x-$x_box_size/2;
$top = $y-$y_box_size/2;
$right = $x+$x_box_size/2;
$bottom = $y+$y_box_size/2;

#print "x: ",$x,"\n";
#print "y: ",$y,"\n";
#print"xbs: ",$x_box_size,", ybs: ",$y_box_size,"\n";
#print $left,",",$top,",",$right,",",$bottom,"\n";

while ( ! defined($img) ) {
  $img = screenshot(left=>$left,right=>$right,top=>$top,bottom=>$bottom) || die("error: $!"); # on linux get the curser position using 'xinput query-state Mouse[1]' or similar
  sleep(0.020);    
}
$newimg = $img->scale(xpixels=>8, ypixels=>8);
undef($img);

for ($row = 0; $row < 8; $row++) {
  $link->write($start_byte);
  @scanline = $newimg->getscanline(y=>$row);
  for ($led = 0; $led < 8; $led++) {
    ($red,$green,$blue,$alpha) = $scanline[$led]->rgba;
    $link->write(pack("C",$row));
    $link->write(pack("C",$led));
    $link->write(pack("C",$red/$color_scaler));
    $link->write(pack("C",$green/$color_scaler));
    $link->write(pack("C",$blue/$color_scaler));
    #print $row,$led,"\n";
    #print unpack("C",$image[$row*3*8+$led*3+0]),"\n";
    #print unpack("C",$image[$row*3*8+$led*3+1]),"\n";
    #print unpack("C",$image[$row*3*8+$led*3+2]),"\n\n";
    sleep(0.001);
  }
  $link->write($stop_byte);
  sleep(0.001);
}

}
