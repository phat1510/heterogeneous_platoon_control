function model = model_estimation(input,output,sampling_time)
    samples = iddata(output,input,sampling_time);
    % leaf_accel = idfrd(samples,'P1D');
    iodelay = NaN;
    model = tfest(samples,1,0,iodelay);
end
