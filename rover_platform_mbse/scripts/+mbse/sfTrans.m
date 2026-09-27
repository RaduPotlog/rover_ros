function t = sfTrans(parent, src, dst, label, order, srcClock, dstClock)
%SFTRANS Add a transition src -> dst with a guard label and execution order.
%   srcClock/dstClock: attachment points (0-12, o'clock) to keep arrows readable.
t = Stateflow.Transition(parent);
t.Source = src;
t.Destination = dst;
if nargin > 5
    t.SourceOClock = srcClock;
    t.DestinationOClock = dstClock;
end
t.LabelString = label;
t.ExecutionOrder = order;
end
