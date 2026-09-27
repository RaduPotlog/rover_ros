function ch = addChart(path, position, sampleTime)
%ADDCHART Add a discrete Stateflow chart (MATLAB action language).
add_block('sflib/Chart', path, 'Position', position);
ch = sfroot().find('-isa', 'Stateflow.Chart', 'Path', path);
ch.ActionLanguage = 'MATLAB';
ch.ChartUpdate = 'DISCRETE';
ch.SampleTime = num2str(sampleTime);
end
