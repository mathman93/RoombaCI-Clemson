
''' Purpose: Store compositions for use in Roomba_SongTesting'''
#work on making file structure cleaner, need to get which composition from user, then which part
tone_mod = -7 # half step modulation of key
rest = 15 - tone_mod # Rest note


DKTest = [72,16,rest,8,74,16,79,8,81,16,rest,8,79,16,rest,8,84,16,rest,8,83,16,79,8,77,16,rest,32,\
                71,16,rest,8,74,16,77,8,83,16,rest,8,81,16,rest,8,80,16,rest,8,79,16,77,8,76,16,rest,32,\
                72,16,rest,8,74,16,79,8,81,16,rest,8,79,16,rest,8,88,16,rest,8,86,16,84,8,81,16,rest,32,\
                81,16,rest,8,83,16,84,8,84,16,79,8,76,16,72,8,78,16,rest,8,77,8,rest,8,76,16,rest,32] # DK64

RickRoll = [72,8,74,8,77,8,74,8,81,24,81,24,79,48,72,8,74,8,77,8,74,8,79,24,79,24,77,16,76,8,74,24,\
                74,8,74,8,77,8,74,8,77,32,79,16,76,24,74,8,72,32,72,16,79,32,77,48,15,16,\
                72,8,74,8,77,8,74,8,81,24,81,24,79,48,72,8,74,8,77,8,74,8,84,32,76,16,77,24,76,8,74,16,\
                74,8,74,8,77,8,74,8,77,32,79,16,76,24,74,8,72,32,72,16,79,32,77,56,15,8] # A rick roll
Comp_dict = {"DK": {"1" : DKTest
                   },
            "RickRoll": {"1" : RickRoll
                        },
             "Rick&DK":{"1" : RickRoll, "2": DKTest}
            }
