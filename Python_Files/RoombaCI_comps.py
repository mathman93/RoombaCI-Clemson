
''' Purpose: Store compositions for use in Roomba_SongTesting'''
#work on making file structure cleaner, need to get which composition from user, then which part
timestep = 8 # (1/64)ths of a second
tone_mod = -7 # half step modulation of key
rest = 15 - tone_mod # Rest note


DKTest = [72,2,rest,1,74,2,79,1,81,2,rest,1,79,2,rest,1,84,2,rest,1,83,2,79,1,77,2,rest,4,\
                71,2,rest,1,74,2,77,1,83,2,rest,1,81,2,rest,1,80,2,rest,1,79,2,77,1,76,2,rest,4,\
                72,2,rest,1,74,2,79,1,81,2,rest,1,79,2,rest,1,88,2,rest,1,86,2,84,1,81,2,rest,4,\
                81,2,rest,1,83,2,84,1,84,2,79,1,76,2,72,1,78,2,rest,1,77,2,rest,1,76,2,rest,4] # Donkey Kong 64 song, composed by Grant Kirkhope

RickRoll = [72,1,74,1,77,1,74,1,81,3,81,3,79,6,72,1,74,1,77,1,74,1,79,3,79,3,77,2,76,1,74,3,\
                74,1,74,1,77,1,74,1,77,4,79,2,76,3,74,1,72,4,72,2,79,4,77,6,rest,2,\
                72,1,74,1,77,1,74,1,81,3,81,3,79,6,72,1,74,1,77,1,74,1,84,4,76,2,77,3,76,1,74,2,\
                74,1,74,1,77,1,74,1,77,4,79,2,76,3,74,1,72,4,72,2,79,4,77,7,rest,1] # Never Gonna Give You Up, composed by Rick Astley
Comp_dict = {"DK": {"1" : DKTest}, "RickRoll": {"1" : RickRoll}}


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
