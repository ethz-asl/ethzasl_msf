function matfig2pgf_menu( varargin )
%Turns the Matfig2PGF menu in the figure windows on and off.
%
%   Usage:
%     matfig2pgf_menu <on/off>
%  
%   Example:
%     matfig2pgf_menu
%     matfig2pgf_menu on
%   Turns the Matfig2PGF menu on.
%
%   Example:
%     matfig2pgf_menu off
%   Turns the Matfig2PGF menu off.
%
%   The Matfig2PGF menu can be turned on by manually executing
%   matfig2pgf_menu every time MATLAB is started. Or you put the command in
%   startup.m (type doc startup.m for more information) to start the
%   Matfig2PGF menu automatically.

if nargin == 0
    turnOn = true;
elseif (nargin == 1) && strcmpi(varargin{1}, 'on')
    turnOn = true;
elseif (nargin == 1) && strcmpi(varargin{1}, 'off')
    turnOn = false;
else
    error('matfig2pgf_menu:InvalidArgument','The supplied argument is invalid');
end

if turnOn
    figHandles = findobj(0, 'Type', 'figure');
    
    for i = 1:length(figHandles)
        h = figHandles(i);
        addMenuToFigure(h);
    end
    
    if ~strcmp(get(0, 'DefaultFigureCreateFcn'), 'matfig2pgf_menu on')
        fprintf('Starting Matfig2PGF menu\n');
        set(0, 'DefaultFigureCreateFcn', 'matfig2pgf_menu on')
    end
else
    figHandles = findobj(0, 'Type', 'figure');
    
    for i = 1:length(figHandles)
        h = figHandles(i);
        removeMenuFromFigure(h);
    end
    
    if strcmp(get(0, 'DefaultFigureCreateFcn'), 'matfig2pgf_menu on')
        fprintf('Stopping Matfig2PGF menu\n');
        set(0, 'DefaultFigureCreateFcn', '')
    end
end
%- end of function matfig2pgf_menu ----------------------------------------



%% FUNCTION ADDMENUTOFIGURE
%
% Adds the Matfig2PGF menu to a figure
%
%--------------------------------------------------------------------------
function addMenuToFigure(fighandle)
% If the figure does not have a 'figure' MenuBar it is most likely not a
% figure with a plot.
if ~strcmp(get(fighandle, 'MenuBar'), 'figure')
    return;
end

% If the menu is already added to this figure, we do not need to
% add it again
h = findobj(fighandle, 'Type', 'uimenu', 'Label', 'Matfig2PGF');
if h
    if strcmpi(get(h, 'Visible'), 'off')
        set(h, 'Visible', 'on');
    end
    return;
end
   
sft = uimenu(fighandle, 'Label', 'Matfig2PGF');
uimenu(sft, 'Label', 'Resize figure', 'Callback', ...
    sprintf('matfig2pgf_gui_options(''resize'', ''fignr'', %d);', fighandle));
uimenu(sft, 'Label', 'Save figure as pgf', 'Callback', ...
    sprintf('matfig2pgf_gui_options(''save'', ''fignr'', %d);', fighandle));
uimenu(sft, 'Label', 'Resize figure and save as pgf', 'Separator', 'on', ...
    'Callback', ...
    sprintf('matfig2pgf_gui_options(''resizesave'', ''fignr'', %d);', fighandle));

%- end of function addMenuToFigure ----------------------------------------



%% FUNCTION REMOVEMENUFROMFIGURE
%
% Removes the Matfig2PGF menu from a figure
%
%--------------------------------------------------------------------------
function removeMenuFromFigure(fighandle)

delete(findobj(fighandle, 'Type', 'uimenu', 'Label', 'Matfig2PGF'));
%- end of function removeMenuFromFigure -----------------------------------
