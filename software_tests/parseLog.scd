// first use
// $ python xthsense_logging.py -n mylogfilename.log -c
// keep it running for a while
//

// BLOCK ZERO TO EXECUTE
(
// change the path where your logs are here - the resulting wave file will be in the same folder:
~basepath = "/Users/williambrent/GitHub/Xth-API/software_tests/";
// change the filename that you want to read out here"
~filename = "xthsense.log";
)
// END OF BLOCK ZERO TO EXECUTE





// BLOCK ONE TO EXECUTE
(
// variables used:
~deltas = List.new;
~prevCur = List.new;
~ringbuffers = List.new;

a = File.new( ~basepath +/+ ~filename, "r" );

// get and interpret the first line:
b = a.getLine;
~prevLine = b.interpret;


(
while( {b.notNil}  ){ // while there are still lines to read in the log
	~curLine = b.interpret; // interpret the current line
	~deltas.add( ~curLine.at([0,1,2]) - ~prevLine.at([0,1,2]) ); // calculate the delta of: (time as measured in python (incoming time), message id, time as measured on the microcontroller (sent time) - wraps around)
	~prevCur.add(~curLine.at( [3,4] ) ); // prevoius and current index in the ringbuffer of this packet
	~ringbuffers.add( ~curLine.copyToEnd(5) ); // add the rest of the data to the list of ringbuffer data
	~prevLine = ~curLine; // previous line becomes current line
	b = a.getLine; // read in the next line
}
);
a.close; // close the file
);
// END OF BLOCK ONE TO EXECUTE




// BLOCK TWO TO EXECUTE
// plot the timing & message loss data
(
~deltas = ~deltas.drop(5); // drop first five (since at startup we have other messages)
[
	~deltas.flop[0], // deltatimes in seconds of incoming message
	~deltas.flop[1].wrap(0,255), // delta msg indices; wraps at 255; should be 1
	~deltas.flop[2].wrap(0,20000) // deltatimes as measured on device (should be around 51; wraps at 20000
].plot
);
// END OF BLOCK TWO TO EXECUTE




// BLOCK THREE TO EXECUTE
(
~prevCur = ~prevCur.drop(5); // same drop as above
~prevCur.collect{ |it| (it[1] - it[0]).wrap(0,63) }.plot; // previous index - current index
~ringbuffers = ~ringbuffers.drop(5); // same drop as above
)
// END OF BLOCK THREE TO EXECUTE




// BLOCK FOUR TO EXECUTE
// calculate the stream of values by reading from the ringbuffers
(
~values = [];
~prevCur.do{ |it,i| // iterate over previous and current indices
	if ( it[1] > it[0] ){ // current index larger than previous index
		~indices = (it[0]..(it[1]-1)); // get the indices we need to read, this is a list
	}{ // else: previous index larger than current index
		~indices = (it[0]..(it[1]+64-1)).collect{|j| j%64}; // get the indices we need to read, this is a list
	};
	~values = ~values ++ ~ringbuffers[i].at( ~indices ); // add the data from the ringbuffer to the array of values
}
);
// END OF BLOCK FOUR TO EXECUTE




// EXECUTE THE LINE
~values.plot; // actual values




// BLOCK FIVE TO EXECUTE
(
// write to a soundfile:
// ~floatValues = ~values - 512 / 512; // between -1 and 1, centering at 0
// 520 seems to be the average: ~values.mean.round(1)
~floatValues = ~values - ~values.mean.round(1) / 512; // between -1 and 1, centering at 0

f = SoundFile.new.headerFormat_( "WAV" ).sampleFormat_( "int16" ).numChannels_(1).sampleRate_(1000);
f.openWrite( ~basepath +/+ "mmgdata_" ++ Date.localtime.stamp ++ ".wav" );
f.writeData( Signal.newFrom( ~floatValues ); );
f.close;
);
// END OF BLOCK FIVE TO EXECUTE
