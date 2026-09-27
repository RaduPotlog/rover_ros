function s = sfState(parent, label, position)
%SFSTATE Add a state. label = name + actions, e.g. sprintf('Idle\nentry: mode = 1;').
s = Stateflow.State(parent);
s.LabelString = label;
s.Position = position;
end
