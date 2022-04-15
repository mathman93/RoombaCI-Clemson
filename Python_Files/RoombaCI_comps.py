
''' Purpose: Store compositions for use in Roomba_SongTesting'''
#work on making file structure cleaner, need to get which composition from user, then which part

DKTest = ["C5",16,"R",8,"D5",16,"G5",8,"A5",16,"R",8,"G5",16,"R",8,"C6",16,"R",8,"B5",16,"G5",8,"F5",16,"R",32,\
		"B4",16,"R",8,"D5",16,"F5",8,"B5",16,"R",8,"A5",16,"R",8,"Ab5",16,"R",8,"G5",16,"F5",8,"E5",16,"R",32,\
		"C5",16,"R",8,"D5",16,"G5",8,"A5",16,"R",8,"G5",16,"R",8,"E6",16,"R",8,"D6",16,"C6",8,"A6",16,"R",32,\
		"A5",16,"R",8,"B5",16,"C5",8,"C5",16,"G5",8,"E5",16,"C5",8,"Gb5",16,"R",8,"F5",8,"R",8,"E5",16,"R",32] # Donkey Kong 64 song, composed by Grant Kirkhope

RickRoll = ["C5",8,"D5",8,"F5",8,"D5",8,"A5",24,"A5",24,"G5",48,"C5",8,"D5",8,"F5",8,"D5",8,"G5",24,"G5",24,"F5",16,"E5",8,"D5",24,\
		"D5",8,"D5",8,"F5",8,"D5",8,"F5",32,"G5",16,"E5",24,"D5",8,"C5",32,"C5",16,"G5",32,"F5",48,"r",16,\
		"C5",8,"D5",8,"F5",8,"D5",8,"A5",24,"A5",24,"G5",48,"C5",8,"D5",8,"F5",8,"D5",8,"C6",32,"E5",16,"F5",24,"E5",8,"D5",16,\
		"D5",8,"D5",8,"F5",8,"D5",8,"F5",32,"G5",16,"E5",24,"D5",8,"C5",32,"C5",16,"G5",32,"F5",56,"r",4] # Never Gonna Give You Up, composed by Rick Astley

AGrace_s = ["C4",4, "F4",8, "A4",2, "F4",2, "A4",8, "G4",4, "F4",8, "D4",4, "C4",8,\
		"C4",4, "F4",8, "A4",2, "F4",2, "A4",8, "G4",4, "C5",20,\
		"A4",4, "C5",6, "A4",2, "C5",2, "A4",2, "F4",8, "C4",4, "D4",6, "F4",2, "F4",2, "D4",2, "C4",8,\
		"C4",4, "F4",8, "A4",2, "F4",2, "A4",8, "G4",4, "F4",20]

SuperStar = ["F5",12,\
			"E5",12]

#class Music:
Comp_dict = {"DK": {"1" : DKTest
					},
			"RickRoll": {"1" : RickRoll
						},
			"AmazingGrace":{"1" : AGrace_s
							#"2" : AGrace_a,
							#"3" : Agrace_t,
							#"4" : Agrace_b
						}
			}

Note_dict = {"R":15, "G1":31, "Ab1":32, "A1":33,"Bb1":34, "B1":35,"C2":36, "Db2":37,
			"D2":38, "Eb2":39, "E2":40,"F2":41,"Gb2":42, "G2":43,"Ab2":44,
			"A2":45, "Bb2":46, "B2":47,"C3":48,"Db3":49, "D3":50, "Eb3":51,
			"E3":52,"F3":53,"Gb3":54,"G3":55,"Ab3":56, "A3":57,"Bb3":58,
			"B3":59,"C4":60,"Db4":61,"D4":62,"Eb4":63,"E4":64,"F4":65,
			"Gb4":66,"G4":67,"Ab4":68,"A4":69,"Bb4":70,"B4":71,"C5":72,
			"Db5":73,"D5":74,"Eb5":75,"E5":76,"F5":77,"Gb5":78,"G5":79,
			"Ab5":80, "A5":81, "Bb5":82, "B5":83,"C6":84,"Db6":85,"D6":86,
			"Eb6":87,"E6":88,"F6":89,"Gb6":90,"G6":91,"Ab6":92,
			"Bb6":94,"B6":95,"C7":96,"Db7":97,"D7":98,"Eb7":99,"E7":100,
			"F7":101,"Gb7":102,"G7":103,"Ab7":104,"A7":105,"Bb7":106,"B7":107}
# End class Music
       
