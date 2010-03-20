#!/usr/bin/perl -TW

use Imager;
use Imager::Screenshot 'screenshot';


my $img = screenshot(left=>820,right=>1270,top=>30,bottom=>300);
my $newimage = $img->scale(xpixels=>8, ypixels=>8);
$newimage->write(file=>'./out.ppm');
