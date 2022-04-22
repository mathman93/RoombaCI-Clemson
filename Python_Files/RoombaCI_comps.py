''' RoombaCI_comps.py
Purpose: To store data for musical compositions for use in Roomba code
May incorporate directly into RoombaCI_lib.py at a later date.
'''
#work on making file structure cleaner, need to get which composition from user, then which part
class Music:
	# Donkey Kong 64 theme, by Grant Kirkhope
	DK64Theme = ["C5",16,"R",8,"D5",16,"G5",8,"A5",16,"R",8,"G5",16,"R",8,"C6",16,"R",8,"B5",16,"G5",8,"F5",16,"R",32,\
				"B4",16,"R",8,"D5",16,"F5",8,"B5",16,"R",8,"A5",16,"R",8,"Ab5",16,"R",8,"G5",16,"F5",8,"E5",16,"R",32,\
				"C5",16,"R",8,"D5",16,"G5",8,"A5",16,"R",8,"G5",16,"R",8,"E6",16,"R",8,"D6",16,"C6",8,"A5",16,"R",32,\
				"A5",16,"R",8,"B5",16,"C6",8,"C6",16,"G5",8,"E5",16,"C5",8,"Gb5",16,"R",8,"F5",8,"R",8,"E5",16,"R",32]

	# Never Gonna Give You Up, by Rick Astley
	RickRoll = ["C5",8,"D5",8,"F5",8,"D5",8,"A5",24,"A5",24,"G5",48,"C5",8,"D5",8,"F5",8,"D5",8,"G5",24,"G5",24,"F5",16,"E5",8,"D5",24,\
				"D5",8,"D5",8,"F5",8,"D5",8,"F5",32,"G5",16,"E5",24,"D5",8,"C5",32,"C5",16,"G5",32,"F5",48,"R",16,\
				"C5",8,"D5",8,"F5",8,"D5",8,"A5",24,"A5",24,"G5",48,"C5",8,"D5",8,"F5",8,"D5",8,"C6",32,"E5",16,"F5",24,"E5",8,"D5",16,\
				"D5",8,"D5",8,"F5",8,"D5",8,"F5",32,"G5",16,"E5",24,"D5",8,"C5",32,"C5",16,"G5",32,"F5",56,"R",4]

	# Super Mario Bros. Theme Introduction, by Koji Kondo
	SMBIntro = ["E5",8, "E5",12, "R",4, "E5",12, "R",4, "C5",8, "E5",12, "R",4, "G5",12, "R",20, "G4",12, "R",20]

	 # Super Mario Bros. Super Star Theme, by Koji Kondo
	SuperStar = ["F5",12,"F5",12,"F5",18,"F5",12,"F5",12,"F5",6,"F5",12,"F5",12,\
				"E5",12,"E5",12,"E5",18,"E5",12,"E5",12,"E5",6,"E5",12,"E5",12]

	# Super Mario Galaxy Start-up Theme
	SMGTheme = ["G5",20, "Ab5",4, "A5",4, "Bb5",4, "B5",4, "C6",24, "R",8, "G4",32, "C5",32, "G5",32, "Gb5",16, "D5",16, "E5",64]

	# All available notes in chromatic sequence (for testing)
	Chromatic = ["G1",8, "Ab1",8, "A1",8, "Bb1",8, "B1",8, "C2",8, "Db2",8, "D2",8, "Eb2",8, "E2",8, "F2",8, "Gb2",8,\
				"G2",8, "Ab2",8, "A2",8, "Bb2",8, "B2",8, "C3",8, "Db3",8, "D3",8, "Eb3",8, "E3",8, "F3",8, "Gb3",8,\
				"G3",8, "Ab3",8, "A3",8, "Bb3",8, "B3",8, "C4",8, "Db4",8, "D4",8, "Eb4",8, "E4",8, "F4",8, "Gb4",8,\
				"G4",8, "Ab4",8, "A4",8, "Bb4",8, "B4",8, "C5",8, "Db5",8, "D5",8, "Eb5",8, "E5",8, "F5",8, "Gb5",8,\
				"G5",8, "Ab5",8, "A5",8, "Bb5",8, "B5",8, "C6",8, "Db6",8, "D6",8, "Eb6",8, "E6",8, "F6",8, "Gb6",8,\
				"G6",8, "Ab6",8, "A6",8, "Bb6",8, "B6",8, "C7",8, "Db7",8, "D7",8, "Eb7",8, "E7",8, "F7",8, "Gb7",8,\
				"G7",8, "Ab7",8, "A7",8, "Bb7",8, "B7",8, "R",8, "B7",8, "Bb7",8, "A7",8, "Ab7",8, "G7",8,\
				"Gb7",8, "F7",8, "E7",8, "Eb7",8, "D7",8, "Db7",8, "C7",8, "B6",8, "Bb6",8, "A6",8, "Ab6",8, "G6",8,\
				"Gb6",8, "F6",8, "E6",8, "Eb6",8, "D6",8, "Db6",8, "C6",8, "B5",8, "Bb5",8, "A5",8, "Ab5",8, "G5",8,\
				"Gb5",8, "F5",8, "E5",8, "Eb5",8, "D5",8, "Db5",8, "C5",8, "B4",8, "Bb4",8, "A4",8, "Ab4",8, "G4",8,\
				"Gb4",8, "F4",8, "E4",8, "Eb4",8, "D4",8, "Db4",8, "C4",8, "B3",8, "Bb3",8, "A3",8, "Ab3",8, "G3",8,\
				"Gb3",8, "F3",8, "E3",8, "Eb3",8, "D3",8, "Db3",8, "C3",8, "B2",8, "Bb2",8, "A2",8, "Ab2",8, "G2",8,\
				"Gb2",8, "F2",8, "E2",8, "Eb2",8, "D2",8, "Db2",8, "C2",8, "B1",8, "Bb1",8, "A1",8, "Ab1",8, "G1",8, "R",8]

	# Amazing Grace, by John Newton, arranged by Timothy Anglea
	AGrace_s = ["C4",32, "F4",64, "A4",16, "F4",16, "A4",64, "G4",32, "F4",64, "D4",32, "C4",64,\
				"C4",32, "F4",64, "A4",16, "F4",16, "A4",64, "G4",32, "C5",128, "R",32,\
				"A4",32, "C5",48, "A4",16, "C5",16, "A4",16, "F4",64, "C4",32, "D4",48, "F4",16, "F4",16, "D4",16, "C4",64,\
				"C4",32, "F4",64, "A4",16, "F4",16, "A4",64, "G4",32, "F4",128, "R",32] # Soprano Line

	AGrace_a = ["C4",32, "A3",64, "C4",16, "A3",16, "C4",64, "C4",32, "Bb3",64, "Bb3",32, "A3",64,\
				"C4",32, "A3",64, "C4",16, "A3",16, "C4",64, "F4",32, "F4",96, "E4",32, "R",32,\
				"C4",32, "F4",48, "C4",16, "F4",16, "C4",16, "A3",64, "A3",32, "Bb3",48, "D4",16, "D4",16, "Bb3",16, "A3",64,\
				"C4",32, "A3",64, "C4",16, "A3",16, "C4",64, "Bb3",32, "Bb3",96, "A3",32, "R",32] # Alto Line

	# Dictionary combining and structuring the previously defined songs
	Comp_dict = {"Demos": {"DK64" : DK64Theme,
						"RickRoll" : RickRoll,
						"SMBIntro": SMBIntro,
						"SuperStar": SuperStar,
						"MarioGalaxy": SMGTheme,
						"AllNotes": Chromatic
						},
				"AmazingGrace":{"1-S" : AGrace_s,
								"2-A" : AGrace_a
								#"3-T" : Agrace_t,
								#"4-B" : Agrace_b
						}
			}

	Note_dict = {"R":15, "G1":31, "Ab1":32, "A1":33, "Bb1":34, "B1":35, "C2":36, "Db2":37,
				"D2":38, "Eb2":39, "E2":40, "F2":41, "Gb2":42, "G2":43, "Ab2":44,
				"A2":45, "Bb2":46, "B2":47, "C3":48, "Db3":49, "D3":50, "Eb3":51,
				"E3":52, "F3":53, "Gb3":54, "G3":55, "Ab3":56, "A3":57, "Bb3":58,
				"B3":59, "C4":60, "Db4":61, "D4":62, "Eb4":63, "E4":64, "F4":65,
				"Gb4":66, "G4":67, "Ab4":68, "A4":69, "Bb4":70, "B4":71, "C5":72,
				"Db5":73, "D5":74, "Eb5":75, "E5":76, "F5":77, "Gb5":78, "G5":79,
				"Ab5":80, "A5":81, "Bb5":82, "B5":83, "C6":84, "Db6":85, "D6":86,
				"Eb6":87, "E6":88, "F6":89, "Gb6":90, "G6":91, "Ab6":92, "A6":93,
				"Bb6":94, "B6":95, "C7":96, "Db7":97, "D7":98, "Eb7":99, "E7":100,
				"F7":101, "Gb7":102, "G7":103, "Ab7":104, "A7":105, "Bb7":106, "B7":107}

	def __init__(self):
		pass
	# End __init__
	# Splits songlist into segments of 16 note-duration pairs for Roomba playback
	def Song_DictCreate(self, songlist):
		songdict = {}
		index = 0
		n = len(songlist)
		while n > 32:
			song, songlist = self.Song_Split(songlist) # Get 16 note-duration pairs for song
			songdict[index] = song # Assign song to dictionary
			index += 1 # Increment counter
			n -= 32 # Update length of songlist
		# End while
		songdict[index] = songlist
		return songdict

	# Separate first 16 note-duration pairs from fullsonglist
	def Song_Split(self, fullsonglist):
		song = fullsonglist[0:32]
		remain = fullsonglist[32:]
		return [song, remain]
	
	# Select composition and part to play on the Roomba
	def Song_Select(self):
		while True:
			print("Here are the available compositions:\n")
			# Display Composition key names:
			complist = sorted(self.Comp_dict.keys())
			disp_str = ""
			for key in complist:
				disp_str = disp_str + key + "; "
			# End for
			print(disp_str + "\n")
			compstr = input("Which composition would you like to play? ")
			if compstr in complist:
				break
			else:
				print("Composition Name not valid. Try again.")
				continue
			# End if
		# End while
		while True:
			print("The available parts for this composition are:\n")
			# Display Part key names:
			partlist = sorted(self.Comp_dict[compstr].keys())
			disp_str = ""
			for key in partlist:
				disp_str = disp_str + key + "; "
			# End for
			print(disp_str + "\n")
			partstr = input("Which part would you like to play? ")
			if partstr in partlist:
				FullSongList = self.Comp_dict[compstr][partstr]
				break
			else:
				print("Part Name not valid. Try again.")
				continue
			# End if
		# End while
# End class Music
       
