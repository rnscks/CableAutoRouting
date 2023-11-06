from enum import Enum
import math

class BitID(Enum):
    DBWORD_BITS_MASK = (8 - 1)
    LOG2_DBWORD_BITS = math.ceil(DBWORD_BITS_MASK) / math.log10(2)
    
print(BitID.DBWORD_BITS_MASK.value)
print(BitID.LOG2_DBWORD_BITS.value)