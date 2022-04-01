
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


Note_dict = {"G1":31, "Ab1":32, "A2":33,"Bb2":34, "B2":35, "C2":36, "Db2":37,
             "D2":38, "Eb2":39, "E2":40,"F2":41,"Gb2":42, "G2":43,"Ab2":44,
             "A3":45, "Bb3":46, "B3":47,"C3":48,"Db3":49,"D3":50,"Eb3":51,
             "E3":52,"F3":53,"Gb3":54,"G3":55,"Ab3":56, "A4":57,"Bb4":58,
             "B4":59,"C4":60,"Db4":61,"D4":62,"Eb4":63,"E4":64,"F4":65,
             "Gb4":66,"G4":67,"Ab4":68,"A5":69,"Bb5":70,"B5":71,"C5":72,
             "Db5":73,"D5":74,"Eb5":75,"E5":76,"F5":77,"Gb5":78,"G5":79,
             "Ab5":80}
