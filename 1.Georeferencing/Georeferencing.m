% Read the ZF file which is transfered to text file
zf_filename = load('');

% Read the INS file (ATLANS) which is in text format
INS_filename = load('');

% Read the metadata
metadata;

% Linear interpolation of INS data with respect to ZF file

% Variables that we need
index;
timestamp;
xLidar;
yLidar;
zLidar;
roll_interp;
pitch_interp;
heading_interp;
rollSd_interp;
pitchSd_interp;
headingSd_interp;
latitude_interp;
longitude_interp;
EllipHeight_interp;
EastingSd_interp;
NorthingSd_interp;
VerticalSd_interp;
