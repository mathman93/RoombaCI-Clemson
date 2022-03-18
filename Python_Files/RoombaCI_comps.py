
''' Purpose: Store compositions for use in Roomba_SongTesting'''
#work on making file structure cleaner, need to get which composition from user, then which part

DKTest = [72,16,15,8,74,16,79,8,81,16,15,8,79,16,15,8,84,16,15,8,83,16,79,8,77,16,15,32,\
		71,16,15,8,74,16,77,8,83,16,15,8,81,16,15,8,80,16,15,8,79,16,77,8,76,16,15,32,\
		72,16,15,8,74,16,79,8,81,16,15,8,79,16,15,8,88,16,15,8,86,16,84,8,81,16,15,32,\
		81,16,15,8,83,16,84,8,84,16,79,8,76,16,72,8,78,16,15,8,77,8,15,8,76,16,15,32] # DK64

RickRoll = [72,8,74,8,77,8,74,8,81,24,81,24,79,48,72,8,74,8,77,8,74,8,79,24,79,24,77,16,76,8,74,24,\
		74,8,74,8,77,8,74,8,77,32,79,16,76,24,74,8,72,32,72,16,79,32,77,48,15,16,\
		72,8,74,8,77,8,74,8,81,24,81,24,79,48,72,8,74,8,77,8,74,8,84,32,76,16,77,24,76,8,74,16,\
		74,8,74,8,77,8,74,8,77,32,79,16,76,24,74,8,72,32,72,16,79,32,77,56,15,8] # A rick roll

class Music:
	Comp_dict = {"DK": {"1" : DKTest
 						},
				"RickRoll": {"1" : RickRoll
						},
				"Rick&DK":{"1" : RickRoll,
							"2": DKTest
							}
				}
# End class Music
       
