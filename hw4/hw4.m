function hw4(questionNum)

    % Load letters dataset (26 letters, A-Z)
    data = load('emnist-letters.mat');
    imgsTrain = permute(reshape(data.dataset.train.images,size(data.dataset.train.images,1),1,28,28),[3,4,2,1]);
    imgsTest = permute(reshape(data.dataset.test.images,size(data.dataset.test.images,1),1,28,28),[3,4,2,1]);
    labelsTrain = categorical(data.dataset.train.labels);
    labelsTest = data.dataset.test.labels;
    trainSize = 40000;
    idxTrain = randperm(size(labelsTrain,1),trainSize);
    imgsTrain = imgsTrain(:,:,:,idxTrain);
    labelsTrain = labelsTrain(idxTrain);

    % Load MNIST digit dataset
    digitDatasetPath = fullfile(matlabroot,'toolbox','nnet','nndemos',...
        'nndatasets','DigitDataset');
    digitData = imageDatastore(digitDatasetPath,...
        'IncludeSubfolders',true,'LabelSource','foldernames');
    trainNumFiles = 750;
    [trainDigitData,valDigitData] = splitEachLabel(digitData,trainNumFiles,'randomize');

    
    % ************* Question 1 *******************
    if questionNum == 1
        
        % Define neural network structure for MNIST training
        layersMNIST = Q1();
        
        % Train deep network
        options = trainingOptions('sgdm',...
        'MaxEpochs',10, ...
        'Verbose',true, ...
        'Plots','training-progress');
        net = trainNetwork(trainDigitData,layersMNIST,options);
        
        % Evaluate accuracy on test set
        predictedLabels = classify(net,valDigitData);
        valLabels = valDigitData.Labels;
        accuracy = sum(predictedLabels == valLabels)/numel(valLabels);
        fprintf('accuracy: %f\n',accuracy)

    end
    
    
    % ************* Question 2 *******************
    if questionNum == 2

        % Define neural network structure
        layersLetters = Q2();

        % Train deep network
        options = trainingOptions('sgdm',...
        'MaxEpochs',1, ...
        'Verbose',true,...
        'Plots','training-progress');
        [net, trinfo] = trainNetwork(imgsTrain,labelsTrain,layersLetters,options);

        % Evaluate accuracy on test set
        predictedLabels = classify(net,imgsTest);
        accuracy = sum(predictedLabels == categorical(labelsTest)) / size(labelsTest,1);
        fprintf('accuracy: %f\n',accuracy)
        
    end
    

    % ************* Question 3 *******************
    if questionNum == 3
        
        % Pretrain on MNIST; finetune on letters
        layersLetters = Q3();
        options = trainingOptions('sgdm',...
        'MaxEpochs',1, ...
        'Verbose',true);
        [net, trinfo_pretrained] = trainNetwork(imgsTrain,labelsTrain,layersLetters,options);
        
        % Compare to training from random weights
        layersLettersRandomStart = Q2();
        [net, trinfo_random] = trainNetwork(imgsTrain,labelsTrain,layersLettersRandomStart,options);

        % Plot learning curve starting from random vs pretrained weights
        figure;
        plot(trinfo_random.TrainingAccuracy,'r');
        hold on;
        plot(trinfo_pretrained.TrainingAccuracy,'b');
        xlabel('iterations');
        ylabel('accuracy');

    end    
    
end

