function h = legend_(varargin)
    % 1. Turn off the 'extra entries' warning and save its previous state
    warnID = 'MATLAB:legend:IgnoringExtraEntries';
    oldWarnState = warning('query', warnID);
    warning('off', warnID);
    
    % 2. Create a cleanup object to restore the warning state when this function finishes
    % (Even if the function errors out, this guarantees the warning turns back on)
    cleanupObj = onCleanup(@() warning(oldWarnState.state, warnID));

    % 3. Call the standard system legend
    [varargout{1:nargout}] = legend(varargin{:});
    
    % 4. Grab the handle
    if nargout > 0
        legHandle = varargout{1};
    else
        legHandle = get(gca, 'Legend');
    end

    % 5. Check for LaTeX setting in a case-insensitive way
    try
        currentDefault = get(groot, 'defaultTextInterpreter');
    catch
        currentDefault = get(0, 'defaultTextInterpreter');
    end
    
    % 6. Apply to legend if default is latex
    if strcmpi(currentDefault, 'latex') && ~isempty(legHandle)
        set(legHandle, 'Interpreter', 'latex');
    end
    
    % 7. Assign output
    if nargout > 0
        h = legHandle;
    end
end