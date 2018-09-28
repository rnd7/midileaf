# MidiLeaf
Arduino App to continously measure the electrical resistance of a plant leaf connected via electrodes. Does some dynamic threshold magic as well as freq analysis. Finally it outputs MIDI Signals to trigger musical instruments.

I coded this primarily for an installation of Katharina Kenklies in 2018.

![drawing](https://raw.githubusercontent.com/rnd7/midileaf/master/doc/drawing.png)

## Wiring and Setup
Using a poor mans breadboard.You can add more electrodes. I tried with 4 pairs, works fine.

![wiring](https://raw.githubusercontent.com/rnd7/midileaf/master/doc/wiring.png)

![prototype](https://raw.githubusercontent.com/rnd7/midileaf/master/doc/prototype.jpg)

![electrodes](https://raw.githubusercontent.com/rnd7/midileaf/master/doc/electrodes.jpg)

![setup](https://raw.githubusercontent.com/rnd7/midileaf/master/doc/setup.png)

## Plot
Using the Arduino IDE you can use the plotter to monitor the values.

![plot](https://raw.githubusercontent.com/rnd7/midileaf/master/doc/plot.png)

## Usage
Sorry no documentation at this point. Read the comments in the code. The app supports multiple electrodes that output multiple voices over discrete MIDI Channels. You can adjust the note ranges and the dynamic threshold parameters, timing and more.

## License

See the [LICENSE](LICENSE.md) file for software license rights and limitations (GPL-v3).
