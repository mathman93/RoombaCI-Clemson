
''' Purpose: Store compositions for use in Roomba_SongTesting'''
#work on making file structure cleaner, need to get which composition from user, then which part
timestep = 8 # (1/64)ths of a second
tone_mod = -7 # half step modulation of key
rest = 15 - tone_mod # Rest note


DKTest = ["C5",16,"r",8,"D5",16,"G5",8,"A5",16,"r",8,"G5",16,"r",8,"C6",16,"r",8,"B5",16,"G5",8,"F5",16,"r",32,\
                "B4",16,"r",8,"D5",16,"F5",8,"B5",16,"r",8,"A5",16,"r",8,"Ab5",16,"r",8,"G5",16,"F5",8,"E5",16,"r",32,\
                "C5",16,"r",8,"D5",16,"G5",8,"A5",16,"r",8,"G5",16,"r",8,"E6",16,"r",8,"D6",16,"C6",8,"A5",16,"r",32,\
                "A5",16,"r",8,"B5",16,"C6",8,"C6",16,"G5",8,"E5",16,"C5",8,"Gb5",16,"r",8,"F5",16,"r",8,"E5",16,"r",32] # Donkey Kong 64 song, composed by Grant Kirkhope

RickRoll = ["C5",8,"D5",8,"F5",8,"D5",8,"A5",24,"A5",24,"G5",48,"C5",8,"D5",8,"F5",8,"D5",8,"G5",24,"G5",24,"F5",16,"E5",8,"D5",24,\
                "D5",8,"D5",8,"F5",8,"D5",8,"F5",32,"G5",16,"E5",24,"D5",8,"C5",32,"C5",16,"G5",32,"F5",48,"r",16,\
                "C5",8,"D5",8,"F5",8,"D5",8,"A5",24,"A5",24,"G5",48,"C5",8,"D5",8,"F5",8,"D5",8,"C6",32,"E5",16,"F5",24,"E5",8,"D5",16,\
                "D5",8,"D5",8,"F5",8,"D5",8,"F5",32,"G5",16,"E5",24,"D5",8,"C5",32,"C5",16,"G5",32,"F5",56,"r",4] # Never Gonna Give You Up, composed by Rick Astley

BaldiSong = ["G4",16,"G5",16,"G4",16,"G5",16,"G4",16,"G5",16,"G4",16,"G5",16,"A4",16,"A5",16,"A4",16,"A5",16,"A4",16,"A5",16,"A4",16,"A5",16] # Hayride Too I Don't Even Care, composed by Cady McGonigal 


VoiceOfPeaceT = ["Bb4",128,"C5",64, "C5", 96, "D5", 32, "C5", 64,"Ab4", 64, "Bb4",192, "C5", 32, "C5", 32, "C5", 32, "Bb4", 32, "C5",32,"D5",32,"Eb5",32,"D5",32,"Eb5",32,"D5",32,"C5",128,"Eb5",64,"G6",32,"F6",32,"D5",32,"C5",32,"D5",32,"F6",32,"G6",64,"D6",64,"Eb6",96,"D6",32,"C6",64,"Ab5",64,"Bb5",256]
                
Comp_dict = {"DK": {"1" : DKTest}, "RickRoll": {"1" : RickRoll}, "Baldi": {"1": BaldiSong}, "VOP":{"1":VoiceOfPeaceT}}


Note_dict = {"r":30, "G1":31, "Ab1":32, "A1":33,"Bb1":34, "B1":35,"C2":36, "Db2":37,
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
