# Mean Filter 

It should be much simpler than this but I wanted to use the inputs packages as variable and the code handle the count for us just by meeting some requirments as below.

By doing so, I had to deal with large latancy, it is about 10 clocks, sure, I could optimize it but I dont want to put more time in it.

By defualt it recive 10 packages, in case you want to modify it do the filowing:

1- You should know what is the number of packages that you will recive so, you can estimate the number of bits for the output port. It is defined as Generic.
2- The total number of packages should be N = (2°n)+2. The 2 belonges to the max and min packages, and this formula should meet cuz we need to deal with devision by 2.

Mean = Sum - max - min.
