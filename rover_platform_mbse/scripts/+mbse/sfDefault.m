function t = sfDefault(parent, dst)
%SFDEFAULT Default transition into dst, drawn from above its top edge.
t = Stateflow.Transition(parent);
t.Destination = dst;
t.DestinationOClock = 0;
p = dst.Position;
x = p(1) + p(3) / 2;
t.SourceEndpoint = [x, p(2) - 25];
t.MidPoint = [x, p(2) - 12];
end
