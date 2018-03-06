% Modify the network from Q1.m to classify the 26 English letters. Your
% network should now have 26 output units.
% 
% For more information, see the LeNet discussion here:
% https://www.mathworks.com/help/nnet/examples/create-simple-deep-learning-network-for-classification.html
%
% input: Null
% output: a matlab "Layer" object. For more info, see:
%         https://www.mathworks.com/help/nnet/ref/nnet.cnn.layer.layer.html
%
function layers = Q2()

    % ** Your code here **

    layers = [
        imageInputLayer([28 28 1])

        convolution2dLayer(3,16,'Padding',1)
        batchNormalizationLayer
        reluLayer

        maxPooling2dLayer(2,'Stride',2)

        convolution2dLayer(3,64,'Padding',1)
        batchNormalizationLayer
        reluLayer

        fullyConnectedLayer(50)
        reluLayer

        fullyConnectedLayer(26)
        softmaxLayer
        classificationLayer];
    
end
