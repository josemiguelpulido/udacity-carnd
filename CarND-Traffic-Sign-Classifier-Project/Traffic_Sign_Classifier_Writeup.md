
## Traffic Sign Recognition Writeup

### Index
The goals / steps of this project are the following:

0. Load the data set (see below for links to the project data set)
1. Explore, summarize and visualize the data set
2. Design, train and test a model architecture
3. Use the model to make predictions on new images
4. Analyze the softmax probabilities of the new images


#### 0. Load the data set

Code to perform this step implemented in cell 34 in Jupyter Notebook, in Step 0 section

#### 1. Explore, summarize and visualize the data set

Code to perform this step implemented in cell 97 in Jupyter Notebook, in Step 1 section.

I have used numpy
to discover various stats about the data. Among them:
- The number of classes is 43
- I was expecting 32x32 to be the shape of the images, but I was not able to display that value

In addition, in cell 36, I have also created code to display the images along with the name of the sign,
to visually confirm that the labels associated to the images were correct, using just a few random cases

I have also included a histogram in cell 9 to show the distribution of traffic signs in the dataset

Furthermore, my mentor suggested to use 
[Stanford cs231 data preprocessing tips](http://cs231n.github.io/neural-networks-2/), so I used mean substraction.
I also tried to apply normalization, by dividing by the standard deviation, but I was not able to make it work.

However, when training the network with mean-substracted data, I was not able to get accuracy beyond 0.5, so I resorted to 
train with unnormalized data. Most probably I am using mean substration wrong.

I also decided not to transform the image into greyscale, because I thought there is value in the color information
avaiable (red vs. blue signals, etc)

#### 2. Design, train and test a model architecture

Code to perform this step implemented in cells 37-42 in Jupyter Notebook, in Step 2 section.

I have used the LetNet network developed during the class as the base ConvNet architecture to classify
the traffic sign images, along with the code the train, validate and test the model:

Input: 32x32x3
Conv: (5x5x3) filter with stride 1 to 28x28x6 output
Relu
Maxpool: (2x2) kernel, (2x2) stride to 14x14x6 output
Conv: (5x5x6) filter with stride 1 to 10x10x16 output
Relu
Maxpool: (2x2) kernel, (2x2) stride to 5x5x16 output
Fully Connected. Input = 400. Output = 120
Relu
Fully Connected. Input = 120. Output = 84
Relu
Fully Connected. Input = 84. Output = 10

I have used a batch size of 128 to implement stochastic gradienrt descent, and 10 epochs.

I have gotten a train/evaluation accuracy of 0.87 and a test accuracy of 0.85. This is with unnormalized data.
I thought that with some preprocessing (e.g., mean substraction) the accuracy would improve, but it did not happen
in my case, probably due to not doing preprocessing correctly. It would be great to get some feedback on this during
the evalution.

I also did not play with modifications to the architecture.



#### 3. Use the model to make predictions on new images

Code to perform this step implemented in cells 37-42 in Jupyter Notebook, in Step 2 section.

Following my mentor suggestion, I have resized images downloaded from the web using cv2. This is implemented in cell 88.
I am not happy with the result of the resize, given that resulting images are distorted. The distorsion happens even
when the original images are square, too. I have spent a significant time on this low level task during the project.
In addition, not getting correct images has not really allowed

For some reason I am not able to iterate across all images. The error I get is displayed in the notebook. It would
be great to get some code examples in Tensorflow on how to compute predictions, as I have not been able to find it
in the class notes.


#### 4. Analyze the softmax probabilities of the new images

Code to perform this step implemented in cell 104 in Jupyter Notebook

In all 5 cases, the top probability is much larger than the rest, indicating a clear candidate. However, the candidate
does not correspond to the actual image

Note that in this case I have been able to iterate across all images w/o problem. I have displayed both the top3 
softmax probabilities for each of the images, along with the name of the sign associated to the top probability in
each case. None of them is correct.

