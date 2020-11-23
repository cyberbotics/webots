import array
import arraycuckoo


# Test built in module
arr = array.array('B', [8])
print(arr)
len(arr)

# Test byte array
barr = bytearray(b'123')

# Test built our module
arr = arraycuckoo.array()
print(arr)
arr.setpointer(barr)
print(arr)
