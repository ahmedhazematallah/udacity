1- Implementation steps for the 2D CFAR process
-----------------------------------------------
- I created a result matrix of size Nr/2 x Nd and initialized it to zeros.
- Then, I used 2 nested loops to loop on the range and doppler dimensions.
- Inside the 2 loops, the noise was calculated by making another 2 for loops accross the training region.
- Guard cells and CUT were excluded, else the value of the signal is accumulated to the noise value.
- After the loop, the noise is divided by the number of the training cells then added the offset.
- Then, the value of CUT was checked versus the noise value.
- To maintain the non threshold region, I started filling the result matrix from index row+Tr+Gr, col+Td+Gd, so  that only the center cells (threshold cells are filled).


2- Selection of Training, Guard cells and offset
------------------------------------------------
- I started with offset value = 3, but I found that there are many 
 unwanted signals everywhere.
- I checked the values of the input of CFAR with the noise and found that they many are near the value of the noise in the corresponding cell.
- I tried incrementing the offset one by one until most of the unwanted signals disappeared.
- Still some unwanted signals existed on the doppler axis. 
- I increased Gd and it became better, then increased Td untill the shape looked good.


3- Steps taken to suppress the non-thresholded cells at the edges
-----------------------------------------------------------------
- Iinitialized the CFAR result matrix by zero. 
- I filled only the the threshold cells inside the loop.
- Therefore, the remaining cells were left with zero.
