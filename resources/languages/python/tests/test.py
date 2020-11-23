import array
import arraymimic


# Test built in module
arr = array.array('B', [8])
print(arr)
len(arr)

# Test built our module
arr = arraymimic.array()
print(arr)
