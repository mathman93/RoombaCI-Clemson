
''' Purpose: Store compositions for use in Roomba_SongTesting'''
#work on making file structure cleaner, need to get which composition from user, then which part
timestep = 8 # (1/64)ths of a second
tone_mod = -7 # half step modulation of key
rest = 15 - tone_mod # Rest note


DKTest = [72,2,rest,1,74,2,79,1,81,2,rest,1,79,2,rest,1,84,2,rest,1,83,2,79,1,77,2,rest,4,\
                71,2,rest,1,74,2,77,1,83,2,rest,1,81,2,rest,1,80,2,rest,1,79,2,77,1,76,2,rest,4,\
                72,2,rest,1,74,2,79,1,81,2,rest,1,79,2,rest,1,88,2,rest,1,86,2,84,1,81,2,rest,4,\
                81,2,rest,1,83,2,84,1,84,2,79,1,76,2,72,1,78,2,rest,1,77,2,rest,1,76,2,rest,4] # DK64

RickRoll = [72,8,74,8,77,8,74,8,81,24,81,24,79,48,72,8,74,8,77,8,74,8,79,24,79,24,77,16,76,8,74,24,\
                74,8,74,8,77,8,74,8,77,32,79,16,76,24,74,8,72,32,72,16,79,32,77,48,15,16,\
                72,1,74,1,77,1,74,1,81,3,81,3,79,6,72,1,74,1,77,1,74,1,84,4,76,2,77,3,76,1,74,2,\
                74,1,74,1,77,1,74,1,77,4,79,2,76,3,74,1,72,4,72,2,79,4,77,7,rest,1] # A rick roll
Comp_dict = {"DK": {"1" : DKTest
                   },
            "RickRoll": {"1" : RickRoll
                        },
             "Rick And DK":{"1" : RickRoll, "2": DKTest}
            }
