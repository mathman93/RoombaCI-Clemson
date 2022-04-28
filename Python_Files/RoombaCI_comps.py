
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

BaldiSong = ["G4",6,"G5",6,"G4",6,"G5",6,"G4",6,"G5",6,"G4",6,"G5",6,"A4",6,"A5",6,"A4",6,"A5",6,"A4",6,"A5",6,"A4",6,"A5",6,\
             "G4",6,"G5",6,"G4",6,"G5",6,"G4",6,"G5",6,"G4",6,"G5",6,"A4",6,"A5",6,"A4",6,"A5",6,"G4",6,"G5",6,"G4",6,"G5",6,\
             "G4",6,"G5",6,"G4",6,"G5",6,"G4",6,"G5",6,"G4",6,"G5",6,"A4",6,"A5",6,"A4",6,"A5",6,"A4",6,"A5",6,"A4",6,"A5",6,\
             "A4",6,"A5",6,"A4",6,"A5",6,"G4",6,"G5",6,"C4",6,"G5",6,"C4",6,"G5",6,"C4",6,"r",6,"G4",6,"D4",6,"C5",6,"r",6] # Hayride Too I Don't Even Care, composed by Cady McGonigal 

# Super Mario Bros. Theme Introduction, by Koji Kondo
SMBIntro = ["E5",8, "E5",12, "r",4, "E5",12, "r",4, "C5",8, "E5",12, "r",4, "G5",12, "r",20, "G4",12, "r",20]
#solos here also pick up harmony parts later
VoiceOfPeaceFSolo = ["r",32,"Eb4",32,"D4",32,"Eb4",32,"Bb4",64,"Eb4",32,"D4",32,"D4",32,"Eb4",32,"C4",96,"r",32,"Eb4",32,"D4",32,"Eb4",32,"Bb4",64,"Eb4",32,"D5",32,"D5",32,"Eb5",32,"C5",32,"Eb5",32,\
                     "Eb5",64,"Ab4",128,"Bb4",32,"Eb5",32,"G4",192,"Eb4",32,"G4",32,"F4",64,"Eb4",128,"Eb4",64,"Eb4",255,"r",1,\
                     "Bb4",192,"r",64,"r",255,"r",3,"Bb4",192,"r",64,"r",255,"r",1,\
                     "r",128,"F4",64,"G4",64,"Eb4",255,"r",1,"r",2,"r",255,"r",1,"r",255,"r",3,\
                     "r",255,"r",1,"Ab4",32,"Ab4",32,"Ab4",192,"r",128,"F3",64,"Eb3",64,"Bb2",255,"r",1,\
                     "r",255,"r",1,"r",255,"r",1,"r",2,"r",255,"r",1,"Ab2",255,"r",1,\
                     "Bb4",192,"r",64,"r",255,"r",3,"Bb4",192,"r",64,"r",255,"r",1,\
                     "r",128,"F4",64,"G4",64,"Eb4",255,"r",1,"r",2,"r",255,"r",1,"Eb2",255,"r",1,\
                     "r",255,"r",1,"Ab2",128,"r",2,"r",128,"r",255,"r",1,"Ab3",128,"G3",64,"F3",64,"r",2,\
                     "r",128,"D4",128,"r",255,"r",1,"r",2,"r",255,"r",1,"C3",64,"Bb2",64\
                     "Ab2",255,"r",1,"r",32,"Eb2",192,"Eb2",255,"r",1] 
VoiceOfPeaceMSolo = ["Bb3",32,"Bb3",64,"Bb3",64,"Eb4",32,"D4",32,"C4",32,"Bb3",32,"Ab3",192,"r",32,"Eb4",32,"D4",32,"C4",32,"Bb3",64,"Bb3",32,"Bb3",32,"C4",32,"Bb3",32,"Ab3",128,"Bb3",32,"G3",32,\
                     "F3",64,"G3",64,"Ab3",64,"Bb3",32,"Ab3",32,"Bb3",192,"Bb3",32,"D4",32,"C4",192,"Ab3",64,"G3",255,"r",1,\
                     "r",255,"r",1,"r",255,"r",1,"r",2,"r",255,"r",1,"r",255,"r",1,"r",255,"r",1,"r",2,"r",255,"r",1,"r",2,"r",255,"r",1,"r",255,"r",2,\
                     "r",255,"r",1,"r",255,"r",1,"r",255,"r",1,"r",255,"r",1,\
                     "r",255,"r",1,"r",255,"r",1,"r",255,"r",1,"r",255,"r",1,\
                     "r",255,"r",1,"r",255,"r",1,"r",255,"r",1,"r",255,"r",1,"r",2,\
                     "Eb2",255,"r",1]
VoiceOfPeaceT = ["G4", 128,"Ab4",64, "Ab4",96, "Bb4",32, "Ab4",64, "F4", 64, "G4", 192,"Ab4",32, "Ab4",32, "Ab4",32, "G4", 32, "Ab4",32, "Bb4",32, "C5", 32, "Bb4",32, "C5", 32, "Bb4",32,\
                           "Ab4",128,"C5", 64, "Eb5",32, "D5", 32, "Bb4",32, "Ab4",32, "Bb4",32, "D5", 32, "Eb5",64, "Bb4",64, "C5", 96, "Bb4",32, "Ab4",64, "F4", 64, "G4", 255,"r",  1,\
                           "G4", 192,"Ab4",32, "Ab4",32, "Ab4",96, "Bb4",32, "Ab4",64, "F4", 64, "r",  2,  "G4", 192,"Ab4",32, "Ab4",32, "Ab4",32, "G4", 32, "Ab4",32, "Bb4",32, "C5", 32, "Bb4",32, "C5", 32, "Bb4",32,\
                           "Ab4",128,"C5", 64, "r",  2,  "Eb5",32, "D5", 32, "Bb4",32, "Ab4",32, "Bb4",32, "D5", 32, "Eb5",64, "Bb4",64 ,"r",  2,  "C5", 96, "Bb4",32, "Ab4",64, "F4", 64, "G4", 255,"r",  1,  "r",  2,\
                           "r",  255,"r",  1,  "r",  128, "C5",64,  "C5", 21, "D5", 21, "Eb5", 21,"r",1,"F5", 64, "Eb5", 64, "C5",64, "C5",32, "Eb5", 32, "Eb5", 128,"D5", 128,\ #triplet here
                           "r",  32, "G4",32, "G4",32, "Ab4", 32, "G4",64, "G4",32, "Bb4", 32, "C5",128,"r",  2,  "C5",64, "Bb4", 32, "Bb4", 32, "C5",64, "Ab4", 64, "C5",64, "F5",32, "Eb5", 32, "Eb5", 64, "D5", 64, "Eb5", 64, "F5",64, \
                           "G5",64, "Eb5", 64, "Bb4", 64, "Ab4", 32, "Ab4", 32, "Ab4", 96, "Bb5", 32, "Ab4", 64, "F4",64, "r",  2,  "G4",192,"Ab4", 32, "Ab4", 32, "Ab4", 32, "G4",32, "Ab4", 32, "Bb4", 32, "C5",32, "Bb4", 32, "C5",32, "Bb4", 32,\
                           "Ab4", 128,"C5",64, "r",  2,  "Eb5", 32, "D5", 32, "Bb4", 32, "Ab4", 32, "Bb4", 32, "D5", 32, "Eb5", 64, "Bb4", 64, "r",  2,  "C5",96, "Bb4", 32, "Ab4", 64, "F4",64, "G4",255,"r",  1,\
                           "Bb4", 192,"Ab4", 64, "Bb4", 128,"r",  2,  "Eb5", 32, "D5", 32, "C5",32, "Bb4", 32, "Bb4", 255,"r",  1,  "Ab4", 128,"G4",64, "F4",64, "r",  2,\
                           "Eb4", 32, "G4",32, "Bb4", 64, "Bb4", 128,"C5",64, "Bb4", 64, "Bb4", 128,"r",  128,"r",  32, "D5", 32, "C5",32, "Bb4", 32, "Ab4", 128,"Bb4", 64, "D5", 64, \
                           "C5",255,"r",  1,  "r",  64, "Bb4", 192,"Bb4", 255,"r",  1] #Voice of Peace Tenor Part
VoiceOfPeaceS = ["r", 255,"r", 255,"r", 255,"r", 255,\ 
                          "r", 255,"r", 255,"r", 255,"r", 255, \
                          "r", 40, "Eb4",32,"D4",32,"Eb4",32,"Bb4",64,"Eb4",32,"D4",32,"D4",32,"Eb4",32,"C4",192,"r",32,"Eb4",32,"D4",32,"Eb4",32,"Bb4",32,"Eb4",32,"D5",32,"D5",32,"Eb5",32,"C5",128,"Bb4",32,"Eb4",32,\ 
                          "Eb5",64,"Ab4",128,"r",2,"Bb4",32,"Eb5",32,"G4",192,"F4",32,"Eb3",128,"r",2,"Eb4",32,"G4",32,"F4",64,"Eb4",128,"Eb4",32,"Eb4",255,"r",1,\
                          "Eb4",64,"G4",32,"Bb4",32,"Eb5",32,"D5",32,"Bb4",32,"G4",32,"Bb4",32,"C5",32,"C5",192,"r",255,"r",1,"r",64,"r",32,"Eb4",32,"D4",32,"Eb4",32,\
                          "Eb5",96,"D5",32,"Bb4",64,"G4",64,"Bb4",64,"C5",128,"Bb4",21,"Ab4",21,"G4",21,"r",1,"G4",32,"F4",160,"r",2,"Ab4",32,"Bb4",32,"C5",96,"D5",32,"Eb5",128,\ #triplet
                          "r",32,"Eb3",32,"D4",32,"Eb4",32,"Bb4",32,"Eb4",32,"D4",32,"Eb4",32,"C4",192,"r",32,"Eb4",32,"D4",32,"Eb4",32,"Bb4",32,"Eb4",32,"D5",32,"D5",32,"Eb5",32,"C5",128,"Bb4",32,"Eb5",32,\
                          "Eb5",64,"Ab4",128,"r",2,"Bb4",32,"Eb5",32,"G4",192,"r",2,"Eb4",32,"G4",32,"F4",64,"Eb4",128,"Eb4",64,"Eb4",255,"r",1,\
                          "Bb4",64,"Ab4",32,"Bb4",32,"Ab4",32,"G4",32,"F4",32,"Eb4",32,"C4",255,"r",1,"r",32,"Bb4",32,"Ab4",32,"Bb4",32,"G4",32,"Eb5",32,"D5",32,"Eb5",32,"Bb4",255,"r",1,"r",2\
                          "Eb5",64,"D5",64,"Bb4",128,"C5",64,"Bb4",64,"G4",128,"r",64,"C5",64,"Bb4",32,"Ab4",32,"C5",32,"D5",64,"Eb5",128,\
                          "Eb5",255,"r",1,"r",32,"G4",192,"G4",255,"r",1]   #Voice of Peace Soprano Part             
VoiceOfPeaceB = ["Eb3", 255, "r", 1, "Ab2", 128, "Ab2",128,"Eb3",255,"r",1,"Ab2",128,"Ab2",64,"Eb3",64,\
                 "F3",255, "r",1, "Eb3",64,"C3",32,"Bb2",32,"Ab2",64,"G2",255,"r",1,"Ab2",128,"Bb2",128,"Eb3",128,\ 
                 "Eb3",255,"r",1,"Ab2",128,"Ab2",128,"r",2,"Eb3",255,"r",1,"Ab2",128,"Ab2",64,"Eb3",64,\
                 "F3",64,"Eb3",64,"C3",32,"Bb2",32,"Ab2",32,"G2",255,"r",1,"r",2,"Ab2",64,"Ab2",64,"Bb2",64,"Bb2",64,"Bb2",255,"r",1"r",2,\
                 "r",255,"r",1,"r",128,"C4",64,"Bb3",21,"Ab3",21,"G3",21,"r",1,"Ab3",96,"G3",32,"Ab3",64,"G3",64,"F3",255,"r",1,\ 
                 "r",32,"C3",32,"C3",32,"C3",64,"Bb2",64,"Ab2",128, "r",2, "Ab3",64,"G3",32,"G3",32,"Ab3",64,"Ab3",32,"G3",32,"F3",32,"Eb3",32,"C3",64,"Eb3",255,"r",1,\
                 "Eb3",255,"r",1,"Ab2",128,"Ab2",128,"r",2,"Eb3",255,"r",1,"Ab2",128,"Ab2",64,"Eb3",64,\
                 "F3",64,"Eb3",64,"C3",32,"Bb2",32,"Ab2",64,"G2",255,"r",1,"r",2,"Ab2",64,"Ab2",64,"Bb2",64,"Bb2",64,"Bb2",255,"r",1,\
                 "Eb3",128,"D3",64,"C3",64,"Eb3",128,"r",2,"C3",64,"D3",64,"Eb3",128,"D3",128,"C3",128,"Bb2",64,"Ab2",64,"r",2,\
                 "G2",32,"Bb2",32,"D3",64,"C3",128,"C3",32,"D3",32,"Eb3",128,"r",255,"r",1,"r",32,"G3",32,"F3",32,"Eb3",32,"C3",128,\
                 "Eb3",255,"r",1,"r",32,"Bb2",192,"D3",255,"r",1]

VoiceOfPeaceA = ["Eb4",128,"D4",64,"Bb3",64,"C4",96,"D4",32,"C4",64,"Ab3",64,"Eb4",128,"D4",64,"Bb3",64,"C4",32,"Bb3",32,"C4",32,"D4",32,"Eb4",32,"D4",32,"Eb4",32,"F4",32,\
                 "F4",32,"Eb4",32,"F4",32,"G4",32,"Ab4",64,"Bb4",64,"G4",96,"F4",32,"Eb4",128,"C4",96,"D4",32,"C4",64,"Ab3",64,"Bb3",255,"r",1,\ #higher split
                 "Eb4",128,"D4",64,"Bb3",32,"Bb3",32,"C4",96,"D4",32,"C4",64,"Ab3",64,"r",2,"Eb4",128,"D4",64,"Bb3",32,"Bb3",32,"C4",32,"Bb3",32,"C4",32,"D4",32,"Eb4",32,"D4",32,"Eb4",32,"F4",32,\
                 "F4",32,"Eb4",32,"F4",32,"G4",32,"Ab4",32,"Bb4",32,"G4",96,"F4",32,"Eb4",128,"r",2,"C4",96,"D4",32,"C4",64,"Ab3",64,"Bb3",255,"r",1,"r",2,\
                 "C4",64,"Eb4",32,"G4",32,"G4",32,"C5",32,"Bb4",32,"G4",32,"Eb4",32,"Eb4",32,"Eb4",32,"Eb4",192,"r",255,"r",1,"r",32,"r",32,"Eb4",32,"D4",32,"D4",32,\
                 "Eb4",32,"Eb4",32,"Eb4",32,"F4",32,"G4",32,"Ab4",32,"G4",32,"F4",32,"Eb4",255,"r",1,"r",2,"C4",64,"C4",64,"Eb4",64,"F4",64,"Bb4",128,"C5",128,\
                 "Eb4",128,"D4",64,"Bb3",32,"Bb3",32,"C4",96,"D4",32,"C4",64,"Ab3",64,"r",2,"Eb4",128,"D4",64,"Bb3",32,"Bb3",32,"C4",32,"Bb4",32,"C4",32,"D4",32,"Eb4",32,"D4",32,"Eb4",32,"F4",32,\
                 "F4",32,"Eb4",32,"F4",32,"G4",32,"Ab4",64,"Bb4",64,"G4",96,"F4",32,"Eb4",128,"r",2,"C4",96,"D4",32,"C4",64,"Bb3",64,"Ab3",255,"r",1,\
                 "G4",64,"F4",32,"G4",32,"F4",32,"Eb4",32,"D4",32,"C4",32,"Bb3",255,"r",1,"r",32,"G4",32,"F4",32,"G4",32,"Eb4",32,"G4",32,"F4",32,"Eb4",255,"r",1,"r",2,\
                 "G4",64,"F4",64,"Eb4",128,"Eb4",64,"D4",32,"F4",32,"Eb4",64,"D4",64,"r",2,"Ab4",64,"G4",32,"F4",32,"Eb4",128,"Eb4",255,"r",1,"Eb4",255,"r",1,"r",32,"Eb4",192,"Eb4",255,"r",1]
Comp_dict = {"SMBIntro": {"1": SMBIntro},
			"DK": {"1" : DKTest},
			"RickRoll": {"1" : RickRoll},
			"Baldi": {"1": BaldiSong},
			"VOP":{"1":VoiceOfPeaceT, "2":VoiceOfPeaceS,"3":VoiceOfPeaceB,"4":VoiceOfPeaceA,"5",VoiceOfPeaceMSolo,"6",VoiceOfPeaceFSolo}}


Note_dict = {"r":30, "G1":31, "Ab1":32, "A1":33, "Bb1":34, "B1":35, "C2":36, "Db2":37,
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
