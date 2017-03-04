# **Backgroud**
# Primary purpose of this very generic message type is to realize
# multi-dimensional list/array with int values in.
# Initial attempt to utilize existing messages such as *MultiArray in
# http://wiki.ros.org/std_msgs didn't go well and it turned out those may not
# be versatile. The same wiki even recommends to create your own message types
# for the complicated data structure. 
int8[] data
