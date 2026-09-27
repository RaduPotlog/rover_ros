function closeAll()
%CLOSEALL Close every model, dictionary, profile, allocation set and requirement
%   set without saving. A "save changes?" dialog would block the MCP session.
bdclose('all');
Simulink.data.dictionary.closeAll('-discard');
systemcomposer.allocation.AllocationSet.closeAll();
systemcomposer.profile.Profile.closeAll();
slreq.clear();
end
