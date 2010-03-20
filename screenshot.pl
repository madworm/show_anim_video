#!/usr/bin/perl -TW

use Imager;
use Imager::Screenshot 'screenshot';


my $img = screenshot();
my $newimage = $img->scale(xpixels=>8, ypixels=>8);
$newimage->write(file=>'./out.ppm');
