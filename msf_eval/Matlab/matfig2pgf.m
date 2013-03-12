function matfig2pgf( varargin )
%MATFIG2PGF  Convert figures to PGF for inclusion in LaTeX documents.
%
%   Matfig2PGF converts a figure to the Portable Graphics Format (PGF).
%   The PGF file can be included in a LaTeX document.
%
%   matfig2pgf(filename)
%   matfig2pgf('option1', option_value1, 'option2', option_value2, ...)
%
%   Example:
%     matfig2pgf('filename', 'figure.pgf', 'fignr', 2, 'figwidth', 5)
%
%   filename - filename where the figure will be stored. If no extension is
%              provided, the .pgf is appended.
%
%   Accepted options:
%     'fignr'
%       uint - number of the figure to convert to pgf. If fignr = 0 then
%       the current figure is used (the gcf command) (default: 0)
%
%     'filename'
%       string - filename where the figure will be stored. If no extension is
%       provided, the .pgf is appended. If no filename is specified a
%       dialog box appears that allows the user to select a file.
%
%     'figwidth'
%       float - width of the figure in cm. The figure height will
%       automatically be calculated, based on the aspect ratio of the
%       figure. If this value is set to zero or less, the figure is not
%       resized. (default: 10)
%
%     'fontname'
%       string - font to be used for all texts in the figure. All text in
%       the figure will be set to this font. Note that this does not
%       specify the actual font used when included in a LaTeX document. In
%       order to get the best result, the font specified here should be as
%       close as possible to the font that will be used in the LaTeX
%       document. (default: 'Times')
%
%     'fontsize'
%       float - size of the fonts for all texts. All text in the figure
%       will be set to this size. This size should be the same as used in
%       the LaTeX document. If this value is zero or less the fonts are not
%       resized. (default: 10)
%
%     'texticklabelsize'
%       string - font size for tick labels in LaTeX output. This value is not
%       used for resizing the figure, it's only used in the LaTeX output.
%       (default: 'normalsize')
%
%     'textextlabelsize'
%       string - font size for all other text labels in LaTeX output. This value
%       is not used for resizing the figure, it's only used in the LaTeX output.
%       (default: 'normalsize')
%
%     'drawfigoutline'
%       boolean - draw a box around the figure (default: false)
%
%     'minlinewidth'
%       float - minimum linewidth in pt. Lines will never be thinner than 
%       this width (default: 0.5pt)
%
%     'includedraftcode'
%       boolean - include a few lines of code in the PGF file. If these
%       lines are added and the pgf package is included using the 'draft'
%       option, then only the axes and labels are drawn. This reduces the
%       compilation time for graphs with a large number of data points.
%       (default: true)
%
%     'textprecode'
%       string - LaTeX code that will be inserted before all text labels in
%       the PGF file. (default: '')
%
%     'textpostcode'
%       string - LaTeX code that will be inserted after all text labels in
%       the PGF file. (default: '')
%
%     'mathprecode'
%       string - LaTeX code that will be inserted before all math code.
%       (default: '$')
%
%     'mathpostcode'
%       string - LaTeX code that will be inserted after all math code.
%      (default: '$')
%
%     'reducedlineerror'
%       float - in order to reduce the size of the PGF file Matfig2PGF will
%       reduce the number of data points, as long as it does not affect the
%       resulting image. If the distance between a data point and the
%       straight line between the previous and next data point that are
%       drawn is less than this value, the data point is left out. This
%       distance is expressed in terms of the line width. A value of 0.1
%       means that the error can not be more than 10% of the line width.
%       (default: 0.1)
%
%     'converttexttolatex'
%       boolean - if set to true Matfig2PGF will try to convert all text
%       strings to proper LaTeX code. E.g. the original string 'x_1' will
%       be converted to '$x_1$'. It is possible to specify your own text
%       that should be used in the PGF file. This can be done by setting
%       the 'UserData' field of the text object. E.g. by
%       set(h, 'UserData', 'matfig2pgf_text: <latex code>');
%       (default: true)
%
%     'snapdistance'
%       float - the position and size of the axes can change, depending on
%       the length of the labels. Especially, the y-axis changes with the
%       ylabel. Compare for example the y-axis position when ylim([0 9])
%       with ylim([0 0.01]). In a report it looks bette when all axes have
%       the same size. Therefore, Matfig2PGF will snap the axes border to
%       values specified in the 'snapverticalborders' and 
%       'snaphorizontalborders' options if they are within the 'snapdistance'.
%       This distance is specified as a normalized value. A value between
%       0 and 1, where (0,0) corresponds to the bottom-left corner of the
%       figure and (1,1) the top-right corner. Note that this can cause the
%       figure to become slightly larger than determined by the 'figwidth'
%       option. (default: 0.1)
%
%     'snapleft'
%       float - snap the top border of axes objects to this distance from
%       the left border of the figure, if they are within 'snapdistance'. This
%       value is relative to font size. The font size, specified in pt, is
%       converted to a normalized value (relative to the figure size). E.g.
%       if the font size = 10pt, figure width = 10cm and snapleft = 5,
%       the normalized snapleft is:
%         10[pt] * 1/72[in/pt] * 2.54[cm/in] / 10[cm] * 5 = 0.1764
%       If set to NaN 'snapleft' is ignored (default: 4)
%
%     'snapright'
%       float - snap the right border of axes objects to this distance from
%       the right border of the figure, if they are within 'snapdistance'. See
%       'snapleft' and 'snapdistance' for more information. If set to NaN
%       'snapright' is ignored (default: 0.8)
%
%     'snapbottom'
%       float - snap the bottom border of axes objects to this distance from
%       the bottom of the figure, if they are within 'snapdistance'. See
%       'snapleft' and 'snapdistance' for more information. If set to NaN
%       'snapbottom is ignored. (default: 3)
%
%     'snaptop'
%       float - snap the top border of axes objects to this distance from
%       the top of the figure, if they are within 'snapdistance'. See
%       'snapleft' and 'snapdistance' for more information. If the axis 
%       object has a title, 'snaptop' is automatically increased by 1. If
%       set to NaN 'snaptop' is ignored. (default: 0.8)
%
%     'noresize'
%       boolean - if set to true the figure is not resized. (default:
%       false)
%
%     'nosave'
%       boolean - if set to true the figure is not saved to file.
%       (default: false)
%
% It is possible to specify your own default options in useroptions.cfg

clear global matfig2pgf_opt;
clear global matfig2pgf_props;
clear global matfig2pgf_version;

global matfig2pgf_version;
matfig2pgf_version = 'Matfig2PGF v0.3.9';

global matfig2pgf_opt;
matfig2pgf_options('set_global', varargin);

fprintf('%s\n', matfig2pgf_version);

% Get the figure handle
if matfig2pgf_opt.fignr > 0
    matfig2pgf_opt.fighandle = matfig2pgf_opt.fignr;
else
    matfig2pgf_opt.fighandle = gcf;
end
fprintf('Figure number: %d\n', matfig2pgf_opt.fighandle);

% Resize the figure
if ~matfig2pgf_opt.noresize
    resize_figure();
end

% Save the figure as pgf to file
if ~matfig2pgf_opt.nosave
    save_to_file();
end

% Try to restore the figure to it's original state
if ~matfig2pgf_opt.keepresizedfigure && ~matfig2pgf_opt.noresize ...
        && ~matfig2pgf_opt.nosave
    restore_remembered_properties
end

clear global matfig2pgf_opt;
clear global matfig2pgf_props;
clear global matfig2pgf_version;

%--------------------------------------------------------------------------



%% FUNCTION RESIZE_FIGURE
%
%  Resize the figure and fonts to match the desired size.
%
%  resize_figure()
%
%--------------------------------------------------------------------------
function resize_figure()
global matfig2pgf_opt;

% Resize figure to correct size
set_and_remember(matfig2pgf_opt.fighandle, 'Units', 'centimeters');
position = get(matfig2pgf_opt.fighandle, 'Position');
if matfig2pgf_opt.figwidth > 0
    matfig2pgf_opt.figheight = matfig2pgf_opt.figwidth * position(4) / position(3);
    fprintf('Figure size: %gcm x %gcm\n', matfig2pgf_opt.figwidth, ...
        matfig2pgf_opt.figheight);
    position(3:4) = [matfig2pgf_opt.figwidth matfig2pgf_opt.figheight];
    set_and_remember(matfig2pgf_opt.fighandle, 'Position', position);
else
    matfig2pgf_opt.figwidth = position(2);
    matfig2pgf_opt.figheight = position(4);
end

% Change the size and name of all fonts, so that they match the LaTeX
% result
if matfig2pgf_opt.fontsize > 0
    fprintf('Font size: %gpt\n', matfig2pgf_opt.fontsize);
    change_fonts(matfig2pgf_opt.fighandle);
end

% Snap the borders of the axes objects, if necessary
if (matfig2pgf_opt.snapdistance > 0)
    snap_axes_borders(matfig2pgf_opt.fighandle);
end

% A small pause makes sure that MATLAB is finished resizing and moving all
% objects
pause(0.01);

%- end of function resize_figure ------------------------------------------



%% FUNCTION SAVE_TO_FILE
%
%  Save the figure as pgf to a file.
%
%  save_to_file()
%
%--------------------------------------------------------------------------
function save_to_file()
global matfig2pgf_opt;
global matfig2pgf_version;

% If no filename is specified, ask the user for one.
if length(matfig2pgf_opt.filename) <= 0
     [file, path] = uiputfile('*.pgf', 'Save figure as...');
     if file == 0
         error('matfig2pgf:save_to_file:CancelledByUser', 'Cancelled by user');
     end
     matfig2pgf_opt.filename = fullfile(path, file);
end
% Check whether the filename has an extension
[pathstr, name, ext] = fileparts(matfig2pgf_opt.filename);
if isempty(ext)
    ext = '.pgf';
end
matfig2pgf_opt.filename = fullfile(pathstr, [name ext]);
fprintf('Saving to: %s\n', matfig2pgf_opt.filename);

% Get the figures dimensions
set_and_remember(matfig2pgf_opt.fighandle, 'Units', 'centimeters');
position = get(matfig2pgf_opt.fighandle, 'Position');
matfig2pgf_opt.figwidth = position(3);
matfig2pgf_opt.figheight = position(4);


fp = fopen(matfig2pgf_opt.filename,'w');
if fp == -1
    error('matfig2pgf:CannotOpenFile','Unable to open %s for writing', matfig2pgf_opt.filename);
end
fprintf(fp, '%% This file has been created by %s\n', matfig2pgf_version);
fprintf(fp, '\\begin{pgfpicture}\n');

% Draw the edge of the figure
if matfig2pgf_opt.drawfigoutline
    fprintf(fp, '  %% Figure outline\n');
    fprintf(fp,'  \\pgfpathrectangle{\\pgfpointorigin}{\\pgfpoint{%gcm}{%gcm}}\n',...
        matfig2pgf_opt.figwidth, matfig2pgf_opt.figheight);
    fprintf(fp,'  \\pgfusepath{stroke}\n');
end

% Draw all children
set_and_remember(0, 'ShowHiddenHandles', 'on');
handle_all_children(matfig2pgf_opt.fighandle, fp);

if matfig2pgf_opt.includedraftcode
    fprintf(fp,'  \\makeatletter\\ifpgf@draftmode\\makeatother');
    fprintf(fp,'\\pgftext[x=%gcm,y=%gcm]{%s\\Huge{DRAFT}%s}\\fi\n', ...
        0.5*matfig2pgf_opt.figwidth, 0.5*matfig2pgf_opt.figheight, ...
        matfig2pgf_opt.textprecode, matfig2pgf_opt.textpostcode);
end
fprintf(fp, '\\end{pgfpicture}\n');
fclose(fp);

%- end of function save_to_file -------------------------------------------



%% FUNCTION HANDLE_ALL_CHILDREN
%
%  Draw all children of a graphics object (if they need to be drawn).
%
%  handle_all_children( handle, fp )
%
%    handle - handle to grsphics object
%    fp     - file pointer to the file were the pgf code is written to
%
%--------------------------------------------------------------------------
function handle_all_children( handle, fp )

child_handles = get(handle, 'Children');
for i = length(child_handles):-1:1
    child_handle = child_handles(i);
    switch get(child_handle, 'Type')
        case 'axes'
            draw_axes(child_handle, fp);
        case 'line'
            draw_line(child_handle, fp);
        case 'text'
            draw_text(child_handle, fp);
        case 'patch'
            draw_patch(child_handle, fp);
        case 'hggroup'
        case 'hgtransform'
        case 'uitoolbar'
        case 'uimenu'
        case 'uicontextmenu'
        case 'uitoggletool'
        case 'uitogglesplittool'
        case 'uipushtool'
        case 'hgjavacomponent'
        otherwise
            fprintf('I don''t know how to handle this object: %s\n', ...
                get(child_handle, 'Type'));
    end
    
    % Also draw all children
    handle_all_children(child_handle, fp);
end

%- end of function handle_all_children ------------------------------------



%% FUNCTION DRAW_PATCH
%
%  Draws a 'patch' graphic object
%
%  draw_patch(handle, fp)
%
%    handle - handle to the patch object
%    fp     - file pointer to were the pgf code must be written
%--------------------------------------------------------------------------
function draw_patch(handle, fp)

global matfig2pgf_opt;

if ~strcmp(get(handle, 'Visible'), 'on')
    return
end

linewidth = get(handle, 'LineWidth');
edgecolor = get(handle, 'EdgeColor');
linestyle = get(handle, 'LineStyle');
facecolor = get(handle, 'FaceColor');
cdata = get(handle, 'CData');

xdata = get(handle, 'XData');
ydata = get(handle, 'YData');

fprintf(fp, '  %% Patch object\n');
fprintf(fp, '  \\begin{pgfscope}\n');
if strcmp(edgecolor, 'flat')
    if size(cdata,3) > 1
        edgecolor = cdata(1,1,:);
    else % indexed colors
        colormap = get(matfig2pgf_opt.fighandle, 'ColorMap');
        if strcmp(get(handle, 'CDataMapping'), 'scaled')
            % need to scale within clim
            clim = get(matfig2pgf_opt.CurrentAxesHandle, 'clim');
            colorindex = round( interp1(clim, ...
				   [1 length(colormap)], cdata(1, 1)) );
        else
            % direct index
            colorindex = cdata(1, 1);
        end;
        edgecolor = colormap(colorindex,:);
    end
end
generate_linestyle_code(fp, linestyle, edgecolor, linewidth, 4);

% Loop through all columns in XData (and YData)
for col = 1:size(xdata,2)
    xydata = transpose([xdata(:,col) ydata(:,col)]);
    % If the parent object is an axes the coordinate unit is 'data',
    % otherwise it is 'normalized'
    parentAxes = get_parent_axes_handle(handle);
    if parentAxes >= 0
        xydata = data2norm(parentAxes, xydata);
    end
    xydata = norm2abs(xydata);

    if strcmp(facecolor, 'flat')
        colormap = get(matfig2pgf_opt.fighandle, 'ColorMap');
        if strcmp(get(handle, 'CDataMapping'), 'scaled')
            % need to scale within clim
            % see MATLAB's manual page for caxis for details
    	    clim = get(matfig2pgf_opt.CurrentAxesHandle, 'clim');
            m = size(colormap,1);
            if (cdata<=clim(1))
        		colorindex = 1;
            elseif (cdata>=clim(2))
        		colorindex = m;
            else
            	colorindex = fix((cdata-clim(1))/(clim(2)-clim(1))*m)+1;
            end
        else
            % direct index
    	    colorindex = cdata(1, 1);
        end
        facecolor = colormap(colorindex,:);
    elseif ( isreal(facecolor) && length(facecolor)==3 )
    else
        error( 'I don''t know how to handle the FaceColor value of %s .', facecolor );
    end
    fprintf(fp, '    \\definecolor{matfig2pgf_facecolor}{rgb}{%g,%g,%g}',...
	facecolor(1), facecolor(2), facecolor(3));
    fprintf(fp, '\\pgfsetfillcolor{matfig2pgf_facecolor}\n');
    
    % Draw the line and fill
    fprintf(fp, '    \\pgfplothandlerlineto\n');
    generate_plotstream_code(fp, xydata, 4);
    fprintf(fp, '    \\pgfpathclose\n');
    if ~strcmp(linestyle, 'none') && ~strcmp(facecolor, 'none')
        fprintf(fp, '    \\pgfusepath{stroke,fill}\n');
    elseif ~strcmp(linestyle, 'none')
        fprintf(fp, '    \\pgfusepath{stroke}\n');
    elseif ~strcmp(facecolor, 'none')
        fprintf(fp, '    \\pgfusepath{fill}\n');
    end
end
fprintf(fp, '  \\end{pgfscope}\n');

% Markers ***TODO***


%- end of function draw_patch ---------------------------------------------


%% FUNCTION DRAW_AXES
%--------------------------------------------------------------------------
function draw_axes(handle, fp)

if ~strcmp(get(handle, 'Visible'), 'on')
    return
end

global matfig2pgf_opt;

% Calculate coordinates of lower-left and upper-right corners of the entire
% axes
position = get(handle, 'Position');
bgcolor = get(handle, 'Color');
x1 = position(1);
x2 = x1+position(3);
y1 = position(2);
y2 = y1+position(4);
coor = norm2abs([x1 x2 ; y1 y2]);
widthheight = norm2abs(transpose(position(3:4)));

fprintf(fp, '  %% Axes\n');
if ~strcmpi(bgcolor, 'none')
    fprintf(fp, '  \\begin{pgfscope}\n');
    fprintf(fp, '    \\definecolor{matfig2pgf_color}{rgb}{%g,%g,%g}', ...
        bgcolor(1), bgcolor(2), bgcolor(3));
    fprintf(fp, '\\pgfsetfillcolor{matfig2pgf_color}\n');
    fprintf(fp, '    \\pgfpathrectangle{\\pgfpoint{%gcm}{%gcm}}{\\pgfpoint{%gcm}{%gcm}}\n',...
        coor(1),coor(2),widthheight(1),widthheight(2));
    fprintf(fp, '    \\pgfusepath{fill}\n');
    fprintf(fp, '  \\end{pgfscope}\n');
end

draw_grid(handle, fp);
draw_tick_marks(handle, fp);

% Draw outer line of the axes
linewidth = max(get(handle, 'LineWidth'), matfig2pgf_opt.minlinewidth);
fprintf(fp,'  %% Outer axes line\n');
box = get(handle, 'Box');
xcolor = get(handle, 'XColor');
ycolor = get(handle, 'YColor');
if strcmp(box, 'on')
    fprintf(fp,'  \\begin{pgfscope}\n');
    fprintf(fp,'    \\pgfsetlinewidth{%.1fpt}\n', linewidth);
    fprintf(fp,'    \\pgfpathrectangle{\\pgfpoint{%gcm}{%gcm}}{\\pgfpoint{%gcm}{%gcm}}\n',...
        coor(1,1),coor(2,1),widthheight(1),widthheight(2));
    fprintf(fp, '    \\pgfusepath{stroke}\n');
    fprintf(fp, '  \\end{pgfscope}\n');
end
if ~strcmp(box, 'on') || any(xcolor ~= [0 0 0])
    % The axes box is not drawn, therefore we need to draw lines for the X-
    % and Y-axis.
    fprintf(fp,'  \\begin{pgfscope}\n');
    fprintf(fp,'    \\pgfsetlinewidth{%.1fpt}\n', linewidth);
    specify_pgf_color(fp, '    ', 'line', xcolor);
    if strcmp(get(handle, 'XAxisLocation'), 'bottom')
         fprintf(fp, '    \\pgfpathmoveto{\\pgfpoint{%fcm}{%fcm}}', ...
             coor(1,1), coor(2,1));
         fprintf(fp, '\\pgfpathlineto{\\pgfpoint{%fcm}{%fcm}}\n', ...
             coor(1,2), coor(2,1));
    else
         fprintf(fp, '    \\pgfpathmoveto{\\pgfpoint{%fcm}{%fcm}}', ...
             coor(1,1), coor(2,2));
         fprintf(fp, '\\pgfpathlineto{\\pgfpoint{%fcm}{%fcm}}\n', ...
             coor(1,2), coor(2,2));         
    end
    fprintf(fp, '    \\pgfusepath{stroke}\n');
    fprintf(fp, '  \\end{pgfscope}\n');
end
if ~strcmp(box, 'on') || any(ycolor ~= [0 0 0])    
    fprintf(fp,'  \\begin{pgfscope}\n');
    fprintf(fp,'    \\pgfsetlinewidth{%.1fpt}\n', linewidth);
    specify_pgf_color(fp, '    ', 'line', ycolor);
    if strcmp(get(handle, 'YAxisLocation'), 'left')
         fprintf(fp, '    \\pgfpathmoveto{\\pgfpoint{%fcm}{%fcm}}', ...
             coor(1,1), coor(2,1));
         fprintf(fp, '\\pgfpathlineto{\\pgfpoint{%fcm}{%fcm}}\n', ...
             coor(1,1), coor(2,2));
    else
         fprintf(fp, '    \\pgfpathmoveto{\\pgfpoint{%fcm}{%fcm}}', ...
             coor(1,2), coor(2,1));
         fprintf(fp, '\\pgfpathlineto{\\pgfpoint{%fcm}{%fcm}}\n', ...
             coor(1,2), coor(2,2));         
    end
    fprintf(fp, '    \\pgfusepath{stroke}\n');
    fprintf(fp, '  \\end{pgfscope}\n');
end

draw_tick_labels(handle, fp);

matfig2pgf_opt.CurrentAxesHandle = handle;

%-- end of draw_axes function ---------------------------------------------



%% FUNCTION DRAW_GRID
%
%    Draw the grid, if the XGrid and/or YGrid property are set 'on'
% 
%    draw_grid( handle, fp )
% 
%--------------------------------------------------------------------------
function draw_grid( handle, fp )

global matfig2pgf_opt;

xgrid = get(handle, 'XGrid');
ygrid = get(handle, 'YGrid');
if strcmp(xgrid,'off') && strcmp(ygrid,'off')
    return
end

position = get(handle, 'Position');
x1 = position(1);
x2 = x1+position(3);
y1 = position(2);
y2 = y1+position(4);
coor = norm2abs([x1 x2 ; y1 y2]);
drawBox = strcmp(get(handle, 'Box'), 'on');
linewidth = max(0.67*get(handle, 'LineWidth'), matfig2pgf_opt.minlinewidth);

fprintf(fp, '  %% Grid\n');
fprintf(fp, '  \\begin{pgfscope}\n');
fprintf(fp, '    \\pgfsetlinewidth{%.1fpt}\n', linewidth);
fprintf(fp, '    \\pgfsetdash{{\\pgflinewidth}{0.5mm}}{0pt}\n');

% Draw the vertical grid lines
if strcmp(xgrid,'on')
    xtick = get_normalized_xtick(handle, drawBox, true);
    if ~isempty(xtick)
        fprintf(fp, '    \\foreach \\x in {');
        for i = 1:length(xtick)
            tempcoor = norm2abs([xtick(i) 0]);
            if i > 1
                fprintf(fp, ',');
            end
            fprintf(fp, '%g', tempcoor(1));
        end
        fprintf(fp, '}\n    {\n');
        fprintf(fp, '      \\pgfpathmoveto{\\pgfpoint{\\x cm}{%gcm}}\\pgfpathlineto{\\pgfpoint{\\x cm}{%gcm}}\n',...
            coor(2,1), coor(2,2));
        fprintf(fp, '    }\n');
    end
end

% Draw the horizontal grid lines
if strcmp(ygrid,'on')
    ytick = get_normalized_ytick(handle, drawBox, true);
    if ~isempty(ytick)
        fprintf(fp, '    \\foreach \\y in {');
        for i = 1:length(ytick)
            tempcoor = norm2abs([0 ytick(i)]);
            if i > 1
                fprintf(fp, ',');
            end
            fprintf(fp, '%g', tempcoor(2));
        end
        fprintf(fp, '}\n    {\n');
        fprintf(fp, '      \\pgfpathmoveto{\\pgfpoint{%gcm}{\\y cm}}\\pgfpathlineto{\\pgfpoint{%gcm}{\\y cm}}\n',...
            coor(1,1), coor(1,2));
        fprintf(fp, '    }\n');
    end
end

fprintf(fp,'    \\pgfusepath{stroke}\n');
fprintf(fp,'  \\end{pgfscope}\n');

%-- end of function draw_grid ---------------------------------------------



%% FUNCTION DRAW_TICK_LABELS
%
%    Draw the tick labels (the numbers near the tick marks)
% 
%    draw_tick_labels( handle, fp )
% 
%--------------------------------------------------------------------------
function draw_tick_labels( handle, fp )

global matfig2pgf_opt;

% Calculate coordinates of lower-left and upper-right corners of the entire
% axes
position = get(handle, 'Position');
x1 = position(1);
x2 = x1 + position(3);
y1 = position(2);
y2 = y1 + position(4);

fprintf(fp, '  %% Tick labels\n');
texticklabelsize = matfig2pgf_opt.texticklabelsize;
if (~isempty(texticklabelsize)) && (~strcmp(texticklabelsize, 'normalsize'))
    fprintf(fp, ['  {\\' texticklabelsize '\n']);
end

% Draw tick labels at x-axis
xticklabels = get(handle, 'XTickLabel');
xtick = get_normalized_xtick(handle);
xtick_data = get(handle, 'XTick');
xloc = get(handle, 'XAxisLocation');
xcolor = get(handle, 'XColor');
isLogScale = strcmp(get(handle, 'XScale'), 'log') && strcmpi(get(handle,'XTickLabelMode'), 'auto');
if strcmpi(xloc, 'top')
    labely = y2;
    textanchor = 'bottom';
    coor2offset = 0.1;
    loganchor = 'bottom';
else
    labely = y1;
    textanchor = 'top';
    coor2offset = -0.1;
    loganchor = 'top';
end
if ~isempty(xticklabels)
    fprintf(fp, '  \\begin{pgfscope}\n');
    specify_pgf_color(fp, '    ', 'fill', xcolor);
    
	multiplier = [];
	for i = 1:length(xtick)
        x = xtick(i);
        coor = norm2abs([x, labely]);
        label = trim_whitespace(xticklabels(1+mod(i-1,length(xticklabels)),:));
        if iscell(label)
            label = cell2mat(label);
        end
        if ~isLogScale
            label_num = str2double(label);
            if ~isempty(label_num) && (label_num ~= 0)
                multiplier = [multiplier xtick_data(i)/label_num];
            end
            fprintf(fp,'    \\pgftext[x=%gcm,y=%gcm,%s]{%s%s%s}\n', ...
                coor(1), coor(2)+coor2offset, textanchor, matfig2pgf_opt.mathprecode, ...
                label, matfig2pgf_opt.mathpostcode);
        else
            fprintf(fp,'    \\pgftext[x=%gcm,y=%gcm,%s]{%s10^{%s}%s}\n', ...
                coor(1), coor(2)+coor2offset, textanchor, matfig2pgf_opt.mathprecode, ...
                label, matfig2pgf_opt.mathpostcode);
        end
	end
	if ~isLogScale && (abs(mean(multiplier)-1) > 1e-10) && strcmpi(get(handle,'XTickLabelMode'), 'auto')
        x = position(1) + position(3);
        y = position(2);
        coor = norm2abs([x y]);
        fprintf(fp,'    \\pgftext[x=%gcm,y=%gcm,left,%s]{%s\\times 10^{%g}%s}\n', ...
            coor(1)+0.2, coor(2), loganchor, matfig2pgf_opt.mathprecode, ...
            log10(mean(multiplier)), matfig2pgf_opt.mathpostcode);
    end
    fprintf(fp, '  \\end{pgfscope}\n');
end

% Draw tick labels at y-axis
yticklabels = get(handle, 'YTickLabel');
ytick = get_normalized_ytick(handle);
ytick_data = get(handle, 'YTick');
yloc = get(handle, 'YAxisLocation');
ycolor = get(handle, 'YColor');
isLogScale = strcmp(get(handle, 'YScale'), 'log') && strcmpi(get(handle,'YTickLabelMode'), 'auto');
if strcmpi(yloc, 'left')
    labelx = x1;
    textanchor = 'right';
    coor1offset = -0.1;
    loganchor = 'right';
else
    labelx = x2;
    textanchor = 'left';
    coor1offset = 0.1;
    loganchor = 'left';
end
if ~isempty(yticklabels)
    fprintf(fp, '  \\begin{pgfscope}\n');
    specify_pgf_color(fp, '    ', 'fill', ycolor);
	multiplier = [];
	for i = 1:length(ytick)
        y = ytick(i);
        coor = norm2abs([labelx, y]);
        label = trim_whitespace(yticklabels(1+mod(i-1,length(yticklabels)),:));
        if iscell(label)
            label = cell2mat(label);
        end
        if ~isLogScale
            label_num = str2double(label);
            if ~isempty(label_num) && (label_num ~= 0)
                multiplier = [multiplier ytick_data(i)/label_num];
            end
            fprintf(fp,'    \\pgftext[x=%gcm,y=%gcm,%s]{%s%s%s}\n', ...
                coor(1)+coor1offset, coor(2), textanchor, matfig2pgf_opt.mathprecode, ...
                label, matfig2pgf_opt.mathpostcode);
        else
            fprintf(fp,'    \\pgftext[x=%gcm,y=%gcm,%s]{%s10^{%s}%s}\n', ...
                coor(1)+coor1offset, coor(2), textanchor, matfig2pgf_opt.mathprecode, ...
                label, matfig2pgf_opt.mathpostcode);
        end
	end
	if ~isLogScale && (abs(mean(multiplier)-1) > 1e-10) && strcmpi(get(handle,'YTickLabelMode'), 'auto')
        coor = norm2abs([labelx y2]);
        fprintf(fp,'   \\pgftext[x=%gcm,y=%gcm,bottom,%s]{%s\\times 10^{%g}%s}\n', ...
            coor(1)+coor1offset, coor(2)+0.1, loganchor, matfig2pgf_opt.mathprecode, ...
            log10(mean(multiplier)), matfig2pgf_opt.mathpostcode);
	end
    fprintf(fp, '  \\end{pgfscope}\n');
end
if (~isempty(texticklabelsize)) && (~strcmp(texticklabelsize, 'normalsize'))
    fprintf(fp, '  }\n');
end

%-- end of draw_tick_labels ------------------------------------------



%% FUNCTION DRAW_TICK_MARKS
%
%    Draw the tick marks
%
%    draw_tick_marks( handle, fp )
%
%--------------------------------------------------------------------------
function draw_tick_marks( handle, fp )

global matfig2pgf_opt;

% DRAW TICK MARKS
ticklength = get(handle, 'TickLength');
ticklength = ticklength(1);
linewidth = max(get(handle, 'LineWidth'), matfig2pgf_opt.minlinewidth);

position = get(handle, 'Position');
x1 = position(1);
x2 = x1+position(3);
y1 = position(2);
y2 = y1+position(4);
coor = norm2abs([x1 x2 ; y1 y2]);

showBox = get(handle, 'Box');
showBox = strcmp(showBox, 'on');
xAxisLocation = get(handle, 'XAxisLocation');
yAxisLocation = get(handle, 'YAxisLocation');
xcolor = get(handle, 'XColor');
ycolor = get(handle, 'YColor');

% Draw the tick marks along x-axis
fprintf(fp, '  %% X tick marks\n');
fprintf(fp, '  \\begin{pgfscope}\n');
fprintf(fp, '    \\pgfsetlinewidth{%.1fpt}\n', linewidth);
specify_pgf_color(fp, '    ', 'line', xcolor);
xtick = get_normalized_xtick(handle, showBox, false);
if ~isempty(xtick)
    fprintf(fp, '    \\foreach \\x in {');
    for x = xtick
        tempcoor = norm2abs([x;0]);
        fprintf(fp, '%g,', tempcoor(1));
    end
    fseek(fp, -1, 'cof');  % remove last comma
    fprintf(fp, '}\n    {\n');
    if showBox || strcmp(xAxisLocation, 'bottom')
        fprintf(fp, '      \\pgfpathmoveto{\\pgfpoint{\\x cm}{%gcm}}\\pgfpathlineto{\\pgfpoint{\\x cm}{%gcm}}\n',...
            coor(2,1), coor(2,1)+ticklength*(coor(2,2)-coor(2,1)));
    end
    if showBox || strcmp(xAxisLocation, 'top')
        fprintf(fp, '      \\pgfpathmoveto{\\pgfpoint{\\x cm}{%gcm}}\\pgfpathlineto{\\pgfpoint{\\x cm}{%gcm}}\n',...
            coor(2,2), coor(2,2)-ticklength*(coor(2,2)-coor(2,1)));
    end
    fprintf(fp, '    }\n');
end

% Draw minor ticks along x-axis
xtick_with_minors = get_normalized_xtick(handle, showBox, true);
if ~isempty(xtick_with_minors)
    if length(xtick_with_minors) > length(xtick)
        fprintf(fp, '    \\foreach \\x in {');
        for x = xtick_with_minors
            % Skip all major tick marks
            if find(x == xtick)
                continue;
            end
            tempcoor = norm2abs([x;0]);
            fprintf(fp, '%g,', tempcoor(1));
        end
        fseek(fp, -1, 'cof');  % remove last comma
        fprintf(fp, '}\n    {\n');
        if showBox || strcmp(xAxisLocation, 'bottom')
            fprintf(fp, '      \\pgfpathmoveto{\\pgfpoint{\\x cm}{%gcm}}\\pgfpathlineto{\\pgfpoint{\\x cm}{%gcm}}\n',...
                coor(2,1), coor(2,1)+0.5*ticklength*(coor(2,2)-coor(2,1)));
        end
        if showBox || strcmp(xAxisLocation, 'top')
            fprintf(fp, '      \\pgfpathmoveto{\\pgfpoint{\\x cm}{%gcm}}\\pgfpathlineto{\\pgfpoint{\\x cm}{%gcm}}\n',...
                coor(2,2), coor(2,2)-0.5*ticklength*(coor(2,2)-coor(2,1)));
        end
        fprintf(fp, '    }\n');
    end
end
fprintf(fp,'    \\pgfusepath{stroke}\n');
fprintf(fp,'  \\end{pgfscope}\n');

% Draw the tick marks along the y-axis
fprintf(fp, '  %% Y tick marks\n');
fprintf(fp, '  \\begin{pgfscope}\n');
fprintf(fp, '    \\pgfsetlinewidth{%.1fpt}\n', linewidth);
specify_pgf_color(fp, '    ', 'line', ycolor);
ytick = get_normalized_ytick(handle, showBox, true);
if ~isempty(ytick)
    fprintf(fp, '    \\foreach \\y in {');
    for y = ytick
        tempcoor = norm2abs([0;y]);
        fprintf(fp, '%g,', tempcoor(2));
    end
    fseek(fp, -1, 'cof');
    fprintf(fp, '}\n    {\n');
    if showBox || strcmp(yAxisLocation, 'left')
        fprintf(fp, '      \\pgfpathmoveto{\\pgfpoint{%gcm}{\\y cm}}\\pgfpathlineto{\\pgfpoint{%gcm}{\\y cm}}\n',...
            coor(1,1), coor(1,1)+ticklength*(coor(1,2)-coor(1,1)));
    end
    if showBox || strcmp(yAxisLocation, 'right')
        fprintf(fp, '      \\pgfpathmoveto{\\pgfpoint{%gcm}{\\y cm}}\\pgfpathlineto{\\pgfpoint{%gcm}{\\y cm}}\n',...
            coor(1,2), coor(1,2)-ticklength*(coor(1,2)-coor(1,1)));
    end
    fprintf(fp, '    }\n');
end

% Draw minor ticks along y-axis
ytick_with_minors = get_normalized_ytick(handle, showBox, true);
if ~isempty(ytick_with_minors)
    if length(ytick_with_minors) > length(ytick)
        fprintf(fp, '    \\foreach \\y in {');
        for y = ytick_with_minors
            % Skip all major tick marks
            if find(y == ytick)
                continue;
            end
            tempcoor = norm2abs([0;y]);
            fprintf(fp, '%g,', tempcoor(2));
        end
        fseek(fp, -1, 'cof');
        fprintf(fp, '}\n    {\n');
        if showBox || strcmp(yAxisLocation, 'left')
            fprintf(fp, '      \\pgfpathmoveto{\\pgfpoint{%gcm}{\\y cm}}\\pgfpathlineto{\\pgfpoint{%gcm}{\\y cm}}\n',...
                coor(1,1), coor(1,1)+0.5*ticklength*(coor(1,2)-coor(1,1)));
        end
        if showBox || strcmp(yAxisLocation, 'right')
            fprintf(fp, '      \\pgfpathmoveto{\\pgfpoint{%gcm}{\\y cm}}\\pgfpathlineto{\\pgfpoint{%gcm}{\\y cm}}\n',...
                coor(1,2), coor(1,2)-0.5*ticklength*(coor(1,2)-coor(1,1)));
        end
        fprintf(fp, '    }\n');
    end
end
fprintf(fp,'    \\pgfusepath{stroke}\n');
fprintf(fp,'  \\end{pgfscope}\n');

%-- end of function draw_tick_marks ---------------------------------------



%% FUNCTION DRAW_TEXT
%
%    draw_text( handle, fp )
%
%    handle - handle to the text object
%    fp     - file identifier
%--------------------------------------------------------------------------
function draw_text( handle, fp )

global matfig2pgf_opt;

if ~strcmp(get(handle, 'Visible'), 'on')
    return
end

% It is possible to defined a different text for usage in LaTeX. This
% different text can be defined in the 'UserData' property of the text.
% This property must contain the field "matfig2pgf_text: <text>".
text = '';
userdata = get(handle, 'UserData');
if ischar(userdata) && ~isempty(userdata)
    fprintf('UserData: %s\n', userdata);
    match = regexp(userdata, 'matfig2pgf_text\s*:\s*(.*)', 'tokenExtents', 'once');
    if ~isempty(match)
        text = userdata(match(1):match(2));
    end
end
if length(text) <= 0
    text = get(handle, 'String');
    if iscell(text)
        text = cell2mat(text);
    end
    if length(trim_whitespace(text)) <= 0
        return
    end
    if matfig2pgf_opt.converttexttolatex
        text = convert_text_to_latex(text);
    end
end

parent_handle = get_parent_axes(handle);
rotation = get(handle, 'Rotation');
halign = get(handle, 'HorizontalAlignment');
valign = get(handle, 'VerticalAlignment');
units = get(handle, 'Units');
color = get(handle, 'Color');

% Get coordinates and convert them to absolute coordinates
set(handle, 'Units', 'data');
position = get(handle, 'Position');
set(handle, 'Units', units);
coor = data2norm(parent_handle, transpose(position(1:2)));
coor = norm2abs(coor);

% Text alignment
switch halign,
    case 'center'
        halign = '';
end
switch valign,
    case 'cap'
        valign = 'top';
    case 'baseline'
        valign = 'base';
    case 'middle'
        valign = '';
end
if (~isempty(valign)) && (~isempty(halign))
    alignText = sprintf('%s,%s,', valign, halign);
elseif ~isempty(valign)
    alignText = sprintf('%s,', valign);
elseif ~isempty(halign)
    alignText = sprintf('%s,', halign);
else
    alignText = '';
end

specify_pgf_color(fp, '    ', 'fill', color);
textextlabelsize = matfig2pgf_opt.textextlabelsize;
if (~isempty(textextlabelsize)) && (~strcmp(textextlabelsize, 'normalsize'))
    fprintf(fp, ['  {\\' textextlabelsize '\n']);
end    
fprintf(fp, '    \\pgftext[%sx=%gcm,y=%gcm,rotate=%g]{%s%s%s}\n', ...
    alignText, coor(1), coor(2), rotation, matfig2pgf_opt.textprecode, ...
    text, matfig2pgf_opt.textpostcode);
if (~isempty(textextlabelsize)) && (~strcmp(textextlabelsize, 'normalsize'))
    fprintf(fp, '  }\n');
end

%-- end of function draw_text ---------------------------------------------



%% FUNCTION CONVERT_TEXT_TO_LATEX
%
%    textout = convert_text_to_line(textin)
%
%       textin  - normal text to be converted to proper latex code
%
%       textout - converted text
%--------------------------------------------------------------------------
function textout = convert_text_to_latex(textin)

global matfig2pgf_opt;

% Split strings in groups separated by space and/or }[a-z]
splitStrings = {};
i = 1;
thisStartIndex = 1;
while i <= length(textin),
    if ~isempty(regexp(textin(i), '\s', 'once'))
        splitStrings{length(splitStrings)+1} = textin(thisStartIndex:i);
        thisStartIndex = i+1;
    elseif (i < length(textin)) && ~isempty(regexpi(textin(i:i+1), '}[a-z]', 'once'))
        splitStrings{length(splitStrings)+1} = textin(thisStartIndex:i);
        thisStartIndex = i+1;
    elseif (i < length(textin)) && ~isempty(regexpi(textin(i:i+1), '[^_\^]{'))
        splitStrings{length(splitStrings)+1} = textin(thisStartIndex:i);
        thisStartIndex = i+1;        
    elseif i == length(textin)
        % Last character of string
        splitStrings{length(splitStrings)+1} = textin(thisStartIndex:i);
    end
    i = i+1;
end

% If two consecutive strings need to set in mathmode and have no whitespace
% in between. They must be joined to one math mode string.
newSplitStrings = {};
for i = 1:length(splitStrings)
    if i > 1
        prev = newSplitStrings{length(newSplitStrings)};
        next = splitStrings{i};
        if inMathMode(prev) && inMathMode(next)
            if isempty(regexp(prev(end), '\s', 'once')) && isempty(regexp(next(1), '\s', 'once'))
                newSplitStrings{length(newSplitStrings)} = [prev next];
            else
                newSplitStrings{length(newSplitStrings)+1} = next;
            end
        else
            newSplitStrings{length(newSplitStrings)+1} = next;
        end
    else
        newSplitStrings{length(newSplitStrings)+1} = splitStrings{i};
    end
end
splitStrings = newSplitStrings;

textout = '';
for i = 1:length(splitStrings)
    if ~inMathMode(splitStrings{i})
        textout = [textout splitStrings{i}];
    else
        thisString = splitStrings{i};
        
        % Remove whitespace at end of string
        lastIndex = length(thisString)+1-regexp(fliplr(thisString), '[^\s]', 'once');
        if lastIndex < length(thisString)
            trailingWhitespace = true;
            thisString = thisString(1:lastIndex);
        else
            trailingWhitespace = false;
        end
        
        % If the are acculades at the beginning and end they can be removed
        if strcmp(thisString(1), '{') && strcmp(thisString(lastIndex), '}')
            thisString = thisString(2:lastIndex-1);
        end

        textout = [textout matfig2pgf_opt.mathprecode thisString ...
            matfig2pgf_opt.mathpostcode];
        if trailingWhitespace
            textout = [textout ' '];
        end
    end
end

% Replace % signs in the text because they are comments in latex
textout = regexprep(textout, '%', '\\%');
%-- end of function convert_text_to_latex ---------------------------------



%% FUNCTION INMATHMODE
%
% Determines whether a string needs to be typeset in math mode in LaTeX
%
% [ mathmode ] = inMathMode( str )
%
%  str - string that needs to be checked
%
%  mathmode - True when it needs to be typeset in math mode, return false
%             when it should be typeset in normal text mode.
function [ mathmode ] = inMathMode( str )
  mathmode = ~isempty(regexp(str, '[|\\_\^]', 'once'));
%-- end of function inmathmode --------------------------------------------



%% FUNCTION DRAW_LINE
%
%    draw_line(handle, fp)
%
%      handle - handle to the line object
%      fp     - file identifier where the pgf commands must be written
%--------------------------------------------------------------------------
function draw_line( handle, fp )

if ~strcmp(get(handle, 'Visible'), 'on')
    return
end

global matfig2pgf_opt;

axes_handle = get_parent_axes(handle);
position = get(axes_handle, 'Position');
axes_x1 = position(1);
axes_x2 = axes_x1+position(3);
axes_y1 = position(2);
axes_y2 = axes_y1+position(4);
axes_coor = norm2abs([axes_x1 axes_x2 ; axes_y1 axes_y2]);
axes_widthheight = norm2abs(transpose(position(3:4)));

xdata = get(handle, 'XData');
fprintf('Drawing line with %d points\n', length(xdata));
ydata = get(handle, 'YData');
normcoor = data2norm(axes_handle, [xdata;ydata]);

visible_groups = find_visible_groups(normcoor, axes_handle);

fprintf(fp, '  %% Line\n');
if matfig2pgf_opt.includedraftcode
    fprintf(fp, '  \\makeatletter\\ifpgf@draftmode\\makeatother\\else\n');
end
fprintf(fp, '  \\begin{pgfscope}\n');
fprintf(fp, '    \\pgfpathrectangle{\\pgfpoint{%gcm}{%gcm}}{\\pgfpoint{%gcm}{%gcm}}\n', ...
    axes_coor(1,1), axes_coor(2,1), axes_widthheight(1,1), axes_widthheight(2,1));
fprintf(fp, '    \\pgfusepath{clip}\n');

% Draw all line parts (groups of points(
for gr = visible_groups
    group = cell2mat(gr);
    draw_line_part(fp, normcoor(:,group.start:group.end), handle);
end

fprintf(fp, '  \\end{pgfscope}\n');
if matfig2pgf_opt.includedraftcode
    fprintf(fp, '  \\fi\n');
end

%- end of function draw_line ----------------------------------------------



%% FUNCTION FIND_VISIBLE_GROUPS
%
%    [ visible_groups ] = find_visible_groups(normcoor, axes_handle)
%
%      normcoor    - normalized coordinates
%      axes_handle - handle of axis object
%
%--------------------------------------------------------------------------
function [ visible_groups ] = find_visible_groups(normcoor, axes_handle)
% Get bounding box in normalized coordinates.
xlim = data2norm_x(axes_handle, get(axes_handle, 'XLim'));
if strcmp(get(axes_handle, 'XDir'), 'reverse')
    xlim = fliplr(xlim);
end
ylim = data2norm_y(axes_handle, get(axes_handle, 'YLim'));
if strcmp(get(axes_handle, 'YDir'), 'reverse')
    ylim = fliplr(ylim);
end

% Check which lines cross the left border of the axes bounding box
leftOfAxis = normcoor(1,:) < xlim(1);
tmp = find(diff(leftOfAxis) ~= 0);
crossingLeft = [];
for i = tmp
    ycross = (xlim(1) - normcoor(1, i)) / (normcoor(1, i + 1) - normcoor(1, i)) ...
        * (normcoor(2, i + 1) - normcoor(2, i)) + normcoor(2, i);
    if (ycross >= ylim(1)) && (ycross <= ylim(2))
        crossingLeft(end + 1) = i; %#ok<AGROW>
    end
end

% Check which lines cross the right border of the axes bounding box
rightOfAxis = normcoor(1,:) > xlim(2);
tmp = find(diff(rightOfAxis) ~= 0);
crossingRight = [];
for i = tmp
    ycross = (xlim(1) - normcoor(1, i)) / (normcoor(1, i + 1) - normcoor(1, i)) ...
        * (normcoor(2, i + 1) - normcoor(2, i)) + normcoor(2, i);
    if (ycross >= ylim(1)) && (ycross <= ylim(2))
        crossingRight(end + 1) = i; %#ok<AGROW>
    end
end

% Check which lines cross the bottom border of the axes bounding box
belowAxis = normcoor(2,:) < ylim(1);
tmp = find(diff(belowAxis) ~= 0);
crossingBottom = [];
for i = tmp
    xcross = (ylim(1) - normcoor(2, i)) / (normcoor(2, i + 1) - normcoor(2, i)) ...
        * (normcoor(1, i + 1) - normcoor(1, i)) + normcoor(1, i);
    if (xcross >= xlim(1)) && (xcross <= xlim(2))
        crossingBottom(end + 1) = i; %#ok<AGROW>
    end
end

% Check which lines cross the bottom border of the axes bounding box
aboveAxis = normcoor(2,:) > ylim(2);
tmp = find(diff(aboveAxis) ~= 0);
crossingTop = [];
for i = tmp
    xcross = (ylim(1) - normcoor(2, i)) / (normcoor(2, i + 1) - normcoor(2, i)) ...
        * (normcoor(1, i + 1) - normcoor(1, i)) + normcoor(1, i);
    if (xcross >= xlim(1)) && (xcross <= xlim(2))
        crossingTop(end + 1) = i; %#ok<AGROW>
    end
end

crossing = union(union(union(crossingLeft, crossingRight), crossingBottom), ...
    crossingTop);
notanumbers = sum(isnan(normcoor));
nandiff = find(diff(notanumbers) ~= 0);
outsideAxis = leftOfAxis + rightOfAxis + belowAxis + aboveAxis;

visible_groups = {};
if notanumbers(1)
    startIdx = nan;
else
    startIdx = 1;
end
N = size(normcoor, 2);
crossing(end + 1) = N;
crossPtr = 1;
nandiff(end + 1) = N;
nanPtr = 1;
while true
    if (crossing(crossPtr) == N) && (nandiff(nanPtr) == N)
        break;
    end
    if crossing(crossPtr) < nandiff(nanPtr)
        % The first crossing is a crossing of the axis bounding box
        idx = crossing(crossPtr);
        crossPtr = crossPtr + 1;
        if outsideAxis(idx) && outsideAxis(idx + 1)
            % we went from outside to outside (crossing through axis)
            group = [idx idx + 1];
            startIdx = nan;        
        elseif outsideAxis(idx)
            % we went from outside to inside
            group = nan;
            startIdx = idx;
        else
            % we went from inside to outside
            group = [startIdx idx + 1];
            startIdx = nan;
        end
    else
        % The first crossing is a crossing from nan to a number or the
        % other way around
        idx = nandiff(nanPtr);
        nanPtr = nanPtr + 1;
        if isnan(startIdx)
            % we went from nan to valid number
            startIdx = idx + 1;
            group = nan;
        else
            % we went from a valid number to nan
            % Check whether the group is inside or outside the axis
            if outsideAxis(idx)
                group = nan;
            else
                group = [startIdx idx];
            end
            startIdx = nan;
        end
    end
    % Add the group to the list if it is visible.
    if ~isnan(group)
        if ~isempty(visible_groups) && (visible_groups{end}.end >= (group(1) - 1))
            % This group adjecent to the previous group
            visible_groups{end}.end = group(2); %#ok<AGROW>
        else
            visible_groups(end + 1) = {struct('start', group(1), 'end', group(2))}; %#ok<AGROW>
        end
    end
end
if ~isnan(startIdx)
        if ~isempty(visible_groups) && (visible_groups{end}.end >= (startIdx - 1))
            % This group adjecent to the previous group
            visible_groups{end}.end = N; %#ok<AGROW>
        else
            visible_groups(end + 1) = {struct('start', startIdx, 'end', N)}; %#ok<AGROW>
        end
end

%%- end of function find_visible_groups -----------------------------------



%% FUNCTION DRAW_LINE_PART
%
%    draw_line_part(fp, xdata, ydata, xlim, ylim, position, handle)
%
%      fp       - file identifier
%      xydata   - array with x- and y-coordinates of the line part that
%                 needs to be drawn
%      handle   - handle to line object
%--------------------------------------------------------------------------
function draw_line_part (fp, xydata, handle)

global matfig2pgf_opt;

linestyle = get(handle, 'LineStyle');
marker = get(handle, 'Marker');
linewidth = max(get(handle, 'LineWidth'), matfig2pgf_opt.minlinewidth);

xydata = norm2abs( xydata );
if strcmp(marker, 'none')
    % Calculate maximum error of reduced line, convert linewidth in pt to cm
    maxerror = matfig2pgf_opt.reducedlineerror * linewidth * (1/72*2.54);
    xydata = reduce_line(xydata, maxerror);
end

% Make sure that values are not too large or too small
xydata(1,:) = min(xydata(1,:), 2*matfig2pgf_opt.figwidth);
xydata(1,:) = max(xydata(1,:), -matfig2pgf_opt.figwidth);
xydata(2,:) = min(xydata(2,:), 2*matfig2pgf_opt.figheight);
xydata(2,:) = max(xydata(2,:), -matfig2pgf_opt.figheight);

% Draw the line itself
if ~strcmp(linestyle, 'none')
    color = get(handle, 'Color');
	
    fprintf(fp, '    %% Draw line part (line itself)\n');
	fprintf(fp, '    \\begin{pgfscope}\n');
    generate_linestyle_code(fp, linestyle, color, linewidth, 6);
	fprintf(fp, '      \\pgfsetroundjoin\n');
	fprintf(fp, '      \\pgfplothandlerlineto\n');
    generate_plotstream_code(fp, xydata, 6);
	fprintf(fp, '      \\pgfusepath{stroke}\n');
	fprintf(fp, '    \\end{pgfscope}\n');
end

% Draw plot marks (if necessary)
if ~strcmp(marker, 'none')
    color = get(handle, 'MarkerEdgeColor');
    if strcmp(color, 'auto')
        color = get(handle, 'Color');
    end

    bgcolor = get(handle, 'MarkerFaceColor');
    markersize = get(handle, 'MarkerSize');

    fprintf(fp, '    %% Draw line part (plot marks)\n');
    fprintf(fp, '    \\begin{pgfscope}\n');
    generate_markerstyle_code(fp, marker, color, linewidth, bgcolor, ...
        markersize, 6);
	generate_plotstream_code(fp, xydata, 6);
	fprintf(fp, '    \\end{pgfscope}\n');
end

%-- end of draw_line_part function ----------------------------------------



%% FUNCTION GENERATE_LINESTYLE_CODE
%
%  Create the code that draws a line
%
%  generate_linestyle_code( linestyle, linecolor, linewidth )
%  generate_linestyle_code( linestyle, linecolor, linewidth, indent )
%
%    fp        - file pointer
%    linestyle - style of the line ('-' solid, '--' dashed, etc...)
%    linecolor - array width rgb values (0-1), color of the line
%    linewidth - width if the line in points
%    indent    - integer, number of spaces to prepend to each line
%
%--------------------------------------------------------------------------
function generate_linestyle_code( fp, linestyle, linecolor, linewidth, ...
    varargin )

% Create the indent with spaces
if nargin > 4
    indent = '';
    for i = 1:varargin{1}
        indent = [indent ' '];
    end
else
    indent = '';
end

% Make sure that the line width is no less than the minimum line width
global matfig2pgf_opt;
linewidth = max(linewidth, matfig2pgf_opt.minlinewidth);

fprintf(fp, '%s\\pgfsetlinewidth{%.2fpt}\n', indent, linewidth);
specify_pgf_color(fp, indent, 'line', linecolor);
switch linestyle
    case '-'
        fprintf(fp, '%s\\pgfsetdash{}{0pt}\n', indent);
    case '--'
        fprintf(fp, '%s\\pgfsetdash{{%.2fpt}{%.2fpt}}{0pt}\n', ...
            indent, 3*linewidth, linewidth);
    case ':'
        fprintf(fp, '%s\\pgfsetdash{{%.2fpt}{%.2fpt}}{0pt}\n', ...
            indent, linewidth, linewidth);
    case '-.'
        fprintf(fp, '%s\\pgfsetdash{{%.2fpt}{%.2fpt}{%.2fpt}{%.2fpt}}{0pt}\n', ...
            indent, linewidth, 2*linewidth, 3*linewidth, 2*linewidth);
end

%- end of function generate_linestyle_code --------------------------------



%% FUNCTION GENERATE_MARKERSTYLE_CODE
%
%  generate_markerstyle_code( fp, marker, markerEdgeColor, markerFaceColor, markerSize)
%  generate_markerstyle_code( fp, marker, markerEdgeColor, markerFaceColor, markerSize, indent )
%
%    fp              - file pointer
%    marker          - string, marker type (i.e. 'o', '+', '*', '.', ...)
%    markeredgecolor - array with rgb values (0-1), color of the marker's
%                      edge
%    markeredgewidth - scalar, width in points of the edge line of the
%                      marker
%    markerfacecolor - array with rgb values (0-1) or 'none', background
%                      color of the marker
%    markersize      - scalar, maker size in points
%    indent          - integer, number of spaces prepended to each line
%                      (default: 0)
%--------------------------------------------------------------------------
function generate_markerstyle_code( fp, marker, markeredgecolor, ...
    markeredgewidth, markerfacecolor, markersize, varargin )

% Create the indent with spaces
if nargin > 4
    indent = '';
    for i = 1:varargin{1}
        indent = [indent ' '];
    end
else
    indent = '';
end

if strcmp(marker, '.')
    markersize = markersize / 3;
end

% Create the pgf code to draw the marker. Store the code that draws a
% marker in the markercode variable
draw_bg = false;
switch marker
    case '+'
        markercode = sprintf('\\pgfpathmoveto{\\pgfpoint{%.2fpt}{0pt}}', -0.5*markersize);
        markercode = sprintf('%s\\pgfpathlineto{\\pgfpoint{%.2fpt}{0pt}}', markercode, 0.5*markersize);
        markercode = sprintf('%s\\pgfpathmoveto{\\pgfpoint{0pt}{%.2fpt}}', markercode, -0.5*markersize);
        markercode = sprintf('%s\\pgfpathlineto{\\pgfpoint{0pt}{%.2fpt}}', markercode, 0.5*markersize);
    case 'o'
        markercode = sprintf('\\pgfpathcircle{\\pgfpointorigin}{%.2fpt}', 0.5*markersize);
        draw_bg = true;
    case '*'
        markercode = sprintf('\\pgfpathmoveto{\\pgfpoint{%.2fpt}{0pt}}', -0.5*markersize);
        markercode = sprintf('%s\\pgfpathlineto{\\pgfpoint{%.2fpt}{0pt}}', markercode, 0.5*markersize);
        markercode = sprintf('%s\\pgfpathmoveto{\\pgfpoint{0pt}{%.2fpt}}', markercode, -0.5*markersize);
        markercode = sprintf('%s\\pgfpathlineto{\\pgfpoint{0pt}{%.2fpt}}', markercode, 0.5*markersize);
        markercode = sprintf('%s\\pgfpathmoveto{\\pgfpoint{%.2fpt}{%.2fpt}}', ...
            markercode, -0.25*sqrt(2)*markersize, -0.25*sqrt(2)*markersize);
        markercode = sprintf('%s\\pgfpathlineto{\\pgfpoint{%.2fpt}{%.2fpt}}', ...
            markercode, 0.25*sqrt(2)*markersize, 0.25*sqrt(2)*markersize);
        markercode = sprintf('%s\\pgfpathmoveto{\\pgfpoint{%.2fpt}{%.2fpt}}', ...
            markercode, -0.25*sqrt(2)*markersize, 0.25*sqrt(2)*markersize);
        markercode = sprintf('%s\\pgfpathlineto{\\pgfpoint{%.2fpt}{%.2fpt}}', ...
            markercode, 0.25*sqrt(2)*markersize, -0.25*sqrt(2)*markersize);
    case '.'
        markercode = sprintf('\\pgfpathcircle{\\pgfpointorigin}{%.2fpt}', 0.5*markersize/3);
        markerfacecolor = markeredgecolor;
        draw_bg = true;
    case 'x'
        markercode = sprintf('\\pgfpathmoveto{\\pgfpoint{%.2fpt}{%.2fpt}}', ...
            -0.25*sqrt(2)*markersize, -0.25*sqrt(2)*markersize);
        markercode = sprintf('%s\\pgfpathlineto{\\pgfpoint{%.2fpt}{%.2fpt}}', ...
            markercode, 0.25*sqrt(2)*markersize, 0.25*sqrt(2)*markersize);
        markercode = sprintf('%s\\pgfpathmoveto{\\pgfpoint{%.2fpt}{%.2fpt}}', ...
            markercode, -0.25*sqrt(2)*markersize, 0.25*sqrt(2)*markersize);
        markercode = sprintf('%s\\pgfpathlineto{\\pgfpoint{%.2fpt}{%.2fpt}}', ...
            markercode, 0.25*sqrt(2)*markersize, -0.25*sqrt(2)*markersize);
    case 'square'
        markercode = sprintf('\\pgfpathrectangle{\\pgfpoint{%.2fpt}{%.2fpt}}{\\pgfpoint{%.2fpt}{%.2fpt}}', ...
            -0.5*markersize, -0.5*markersize, markersize, markersize);
        draw_bg = true;
    case 'diamond'
        markercode = sprintf('\\pgfpathmoveto{\\pgfpoint{%.2fpt}{0pt}}', ...
            -0.5*2/3*markersize);
        markercode = sprintf('%s\\pgfpathlineto{\\pgfpoint{0pt}{%.2fpt}}', ...
            markercode, 0.5*markersize);
        markercode = sprintf('%s\\pgfpathlineto{\\pgfpoint{%.2fpt}{0pt}}', ...
            markercode, 0.5*2/3*markersize);
        markercode = sprintf('%s\\pgfpathlineto{\\pgfpoint{0pt}{%.2fpt}}', ...
            markercode, -0.5*markersize);
        markercode = sprintf('%s\\pgfpathclose', markercode);
        draw_bg = true;
    case {'^','v','>','<'}
        switch marker
            case '^'
                a = -pi/6;
            case 'v'
                a = pi/6;
            case '<'
                a = pi/3;
            case '>'
                a = 0;
        end
        markercode = sprintf('\\pgfpathmoveto{\\pgfpoint{%.2fpt}{%.2fpt}}', ...
            cos(a)*0.5*markersize, sin(a)*0.5*markersize);
        markercode = sprintf('%s\\pgfpathlineto{\\pgfpoint{%.2fpt}{%.2fpt}}', ...
            markercode, cos(a+2*pi/3)*0.5*markersize, sin(a+2*pi/3)*0.5*markersize);
        markercode = sprintf('%s\\pgfpathlineto{\\pgfpoint{%.2fpt}{%.2fpt}}', ...
            markercode, cos(a+4*pi/3)*0.5*markersize, sin(a+4*pi/3)*0.5*markersize);
        markercode = sprintf('%s\\pgfpathclose', markercode);
        draw_bg = true;
    case 'pentagram'
        a = pi/2;
        da = 2*pi/10;
        markercode = sprintf('\\pgfpathmoveto{\\pgfpoint{%.2fpt}{%.2fpt}}', ...
            cos(a)*0.5*markersize, sin(a)*0.5*markersize);
        for i = a+da:2*da:2*pi+a-da;
            markercode = sprintf('%s\\pgfpathlineto{\\pgfpoint{%.2fpt}{%.2fpt}}', ...
                markercode, cos(i)*0.19*markersize, sin(i)*0.19*markersize);
            markercode = sprintf('%s\\pgfpathlineto{\\pgfpoint{%.2fpt}{%.2fpt}}', ...
                markercode, cos(i+da)*0.5*markersize, sin(i+da)*0.5*markersize);
        end
        markercode = sprintf('%s\\pgfpathclose', markercode);
        draw_bg = true;
    case 'hexagram'
        a = pi/2;
        da = 2*pi/12;
        markercode = sprintf('\\pgfpathmoveto{\\pgfpoint{%.2fpt}{%.2fpt}}', ...
            cos(a)*0.5*markersize, sin(a)*0.5*markersize);
        for i = a+da:2*da:2*pi+a-da;
            markercode = sprintf('%s\\pgfpathlineto{\\pgfpoint{%.2fpt}{%.2fpt}}', ...
                markercode, cos(i)*0.29*markersize, sin(i)*0.29*markersize);
            markercode = sprintf('%s\\pgfpathlineto{\\pgfpoint{%.2fpt}{%.2fpt}}', ...
                markercode, cos(i+da)*0.5*markersize, sin(i+da)*0.5*markersize);
        end
        markercode = sprintf('%s\\pgfpathclose', markercode);
        draw_bg = true;
    otherwise
        disp(marker)
end
if strcmp(markerfacecolor, 'none')
    draw_bg = false;
end

fprintf(fp, '%s\\pgfsetlinewidth{%.2fpt}\n', indent, markeredgewidth);
specify_pgf_color(fp, indent, 'line', markeredgecolor);

if draw_bg
    specify_pgf_color(fp, indent, 'fill', markerfacecolor);
    fprintf(fp, '%s\\pgfplothandlermark{%s\\pgfusepath{stroke,fill}}\n', ...
        indent, markercode);
else
    fprintf(fp, '%s\\pgfplothandlermark{%s\\pgfusepath{stroke}}\n', ...
        indent, markercode);
end

%- end of function generate_markerstyle_code --------------------------------



%% FUNCTION GENERATE_PLOTSTREAM_CODE
%
%  Generate a plot stream using xy-coordinates
%
%  generate_plotstream_code( fp, xydata )
%  generate_plotstream_code( fp, xydata, indent )
%
%    fp     - file pointer
%    xydata - array with x- and y-coordinates
%    indent - integer, number of spaces prepended to each line (default: 0)
%
%--------------------------------------------------------------------------
function generate_plotstream_code( fp, xydata, varargin )

% Create the indent with spaces
if nargin > 4
    indent = '';
    for i = 1:varargin{1}
        indent = [indent ' '];
    end
else
    indent = '';
end

if size(xydata,2) < 1
    return
end

fprintf(fp, '%s\\pgfplotstreamstart\n', indent);
fprintf(fp, '%s\\foreach \\x/\\y in {', indent);

if all( isfinite(xydata(:,1)) )
    fprintf(fp, '%.3f/%.3f', xydata(1,1), xydata(2,1));
end
for i = 2:size(xydata,2)
    if any(~isfinite(xydata(:,i)))
        continue
    end
    fprintf(fp, ',%.3f/%.3f', xydata(1,i), xydata(2,i));
    if rem(i, 10) == 9
        fprintf(fp, '\n');
    end
end

fprintf(fp, '}\n%s{\n', indent);
fprintf(fp, '%s\\pgfplotstreampoint{\\pgfpoint{\\x cm}{\\y cm}}\n', indent);
fprintf(fp, '%s}\n', indent);
fprintf(fp, '%s\\pgfplotstreamend\n', indent);

%- end of function generate_plotstream_code -------------------------------



%% FUNCTION REDUCE_LINE
%
%  Remove points that hardly changes the appearance of the line. This will
%  decrease the file size and increase LaTeX compilation time.
%
%  [ reduced_xydata ] = reduce_line( xydata )
%
%      xydata   - array with absolute x- and y-coordinates of the line
%                 part that needs to be drawn (in centimeters)
%
%      reduced_xydata - array with reduced number of normalized x- and
%                       y-coordinates (in centimeters)
%
%--------------------------------------------------------------------------
function [ reduced_xydata ] = reduce_line( xydata, maxerror )

N = size(xydata,2);

if (maxerror <= 0) || (N < 3)
    reduced_xydata = xydata;
    return
end

xydata = minmaxdecimation(xydata, 4*maxerror);

N = size(xydata,2);

plotPoints = 1;
lastPlotted = 1;

xdata = xydata(1,:);
ydata = xydata(2,:);

for i = 3:N
    % Calculate distance
    % see http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
    % x1 = xydata(:,i)  x2 = xydata(:,lastPlotted)  x0 = all points
    % between x1 and x2
    % See also: http://astronomy.swin.edu.au/~pbourke/geometry/pointline/
    p1 = xydata(:,i);
    p2 = xydata(:,lastPlotted);
    dp = sqrt(sum((p1-p2).^2));
    frac1 = ( xdata(lastPlotted+1:i-1)-xdata(i) ) .* ( xdata(lastPlotted)-xdata(i) );
    frac1 = frac1 + ( ydata(lastPlotted+1:i-1)-ydata(i) ) .* ( ydata(lastPlotted)-ydata(i) );
    u = frac1 ./ sum((p2-p1).^2);
    
    % Convert u to the distance from the point to p1 or p2
    % For points where the closest point on line p1-p2 is outside of p1 and
    % p2 u is now the distance to p1 or p2. When the closest point was
    % between p1 and p2 u will be zero.
    u((u > 0) & (u <= 1)) = 0;
    u(u>1) = dp*(u(u>1)-1);
    u(u<0) = dp*(-u(u<0));

    % Calculate shortest distance from point to line (p1-p2)
    a = xdata(lastPlotted)-xdata(i);
    b = ydata(lastPlotted)-ydata(i);
    c = xdata(i)-xdata(lastPlotted+1:i-1);
    d = ydata(i)-ydata(lastPlotted+1:i-1);
    frac1 = abs(a.*d-b.*c);
    frac2 = sqrt(sum((xydata(:,lastPlotted)-xydata(:,i)).^2));
    d = frac1./frac2;

    d = sqrt(d.^2 + u.^2);

    if max(d) > maxerror
        lastPlotted = i-1;
        plotPoints = [plotPoints lastPlotted];
    end
    
end

plotPoints = [plotPoints N];
if N > 5
    reduced_xydata = xydata(:,plotPoints);
else
    reduced_xydata = xydata;
end
if N ~= size(reduced_xydata,2)
    fprintf('  reduced number of data points: original: %d -> reduced: %d\n',...
        N, size(reduced_xydata,2));
end

%-- end of function reduce_line -------------------------------------------



%% FUNCTION MINMAXDECIMATION
%
%  If the number of data points is (much) larger than viewable on screen or
%  paper. On paper the linewidth is the limiting factor. E.g. a line going
%  up and down ten times within the width of the line will only show a
%  single line.
%  Note: this method only works correctly if the x-values are strictly
%  non-decreasing or non-increasing.
%
%  ( decimatedXydata ) = minmaxdecimation( xydata, columnWidth )
%
%    xydata      - array with the x- and y-coordinates
%    columnWidth - width of a "colum" that only the minimum and maximum
%                  coordinate and not all data points (typically something
%                  like 0.25 * linewidth)
%
function [ decimatedXydata ] = minmaxdecimation( xydata, columnWidth )

xdata = xydata(1,:);
ydata = xydata(2,:);
minx = min(xdata);
maxx = max(xdata);
N = (maxx-minx)/columnWidth;  % number of columns

% dx = xdata(i+1)-xdata(i) is the same for all values. Otherwise this
% function is not going to work.
dx = diff(xdata);
maxdx = max(dx);
mindx = min(dx);
dx = median(dx);

% If dx is not the same for all values OR
% if the number of columns is less than 0.5*data length
% then we can not use min-max decimation
if ( (maxdx-mindx)/dx > 1e-6 ) || ( N > 0.5*length(xdata) )
    decimatedXydata = xydata;
    return;
end

decimatedXydata = [];
lastIndex = 0;
for i = 1:ceil(N)
    thisIndex = min( floor(i*columnWidth/dx), length(xdata) );
    [miny, minIndex] = min(ydata(lastIndex+1:thisIndex));
    [maxy, maxIndex] = max(ydata(lastIndex+1:thisIndex));
    minIndex = minIndex+lastIndex;
    maxIndex = maxIndex+lastIndex;
    if minIndex < maxIndex
        decimatedXydata = [decimatedXydata [xdata(minIndex);miny] [xdata(maxIndex);maxy]];
    else
        decimatedXydata = [decimatedXydata [xdata(maxIndex);maxy] [xdata(minIndex);miny]];
    end
    lastIndex = thisIndex;
end
if size(decimatedXydata,2) > 10
    fprintf('  min-max decimation: original %d  decimated %d\n', size(xydata,2), size(decimatedXydata,2));
else
    decimatedXydata = xydata;
end
%- end of function minmaxdecimation ---------------------------------------



%% FUNCTION CHANGE_FONTS
%
%  Change the size and name of all texts
%
%  change_fonts( handle )
%
%    handle - handle to the figure object for which all fonts must be
%             changed
%
%--------------------------------------------------------------------------
function change_fonts( handle )

global matfig2pgf_opt;

texthandles = transpose(findobj(handle, 'Type', 'text'));
axeshandles = transpose(findobj(handle, 'Type', 'axes'));

for h = axeshandles
    if ~isempty(get(get(h, 'XLabel'), 'String') > 0)
        texthandles = [texthandles get(h, 'XLabel')];
    end
    if ~isempty(get(get(h, 'YLabel'), 'String') > 0)
        texthandles = [texthandles get(h, 'YLabel')];
    end
    if ~isempty(get(get(h, 'ZLabel'), 'String') > 0)
        texthandles = [texthandles get(h, 'ZLabel')];
    end
    if ~isempty(get(get(h, 'Title'), 'String') > 0)
        texthandles = [texthandles get(h, 'Title')];
    end
    
    set_and_remember(h, 'FontSize', matfig2pgf_opt.fontsize);
    set_and_remember(h, 'FontName', matfig2pgf_opt.fontname);
end

% In newer MATLAB version findobj(h, 'Type', 'text') will also return the
% Xlabels, Ylabels, Zlabels and titles of axes, while older version don't.
texthandles = unique(texthandles);

for h = texthandles
    set_and_remember(h, 'FontSize', matfig2pgf_opt.fontsize);
    set_and_remember(h, 'FontName', matfig2pgf_opt.fontname);
end

%-- end of function change_fonts ------------------------------------------



%% FUNCTION NORM2ABS
%
%  Convert normalized coordinates to absolute coordinates (of drawing
%  canvas)
%
%  [ abs_coor ] = norm2abs( norm_coor )
%
%    norm_coor - array with x- and y-coordinate. y-coordinate on row 1,
%                y-coordinates on row 2. E.g. norm_coor(5,2) is the y-
%                coordinate of the 5th coordinate.
%
%    abs_coor  - array with absolute (as on drawing canvas) coordinates
%
%--------------------------------------------------------------------------
function [ abs_coor ] = norm2abs( norm_coor )

% Make sure that the x-data is on the first row and the y-data on the
% second row
if (size(norm_coor,1) ~= 2) && (size(norm_coor,2) == 2)
    norm_coor = transpose(norm_coor);
end

global matfig2pgf_opt

mult = [ matfig2pgf_opt.figwidth 0 ; 0 matfig2pgf_opt.figheight ];
abs_coor = mult*norm_coor;

%-- end of function norm2abs ----------------------------------------------



%% FUNCTION DATA2NORM
%
%    [ norm_coor ] = data2norm( handle, data_coor )
%
%    handle    - handle to axes object
%    data_coor - array with x- and y-coordinates, row 1 contains the
%                x-coordinates and row 2 contains the y-coordinates
%
%    norm_coor    - array with normalized x- and y-coordinates
%--------------------------------------------------------------------------
function [ norm_coor, varargout ] = data2norm( handle, data_coor )

% Make sure that the x-data is on the first row and the y-data on the
% second row
if (size(data_coor,1) ~= 2) && (size(data_coor,2) == 2)
    data_coor = transpose(data_coor);
end

norm_x = data2norm_x(handle, data_coor(1,:));
norm_y = data2norm_y(handle, data_coor(2,:));

norm_coor = [ norm_x ; norm_y ];

%- end of function data2norm ----------------------------------------------



%% FUNCTION DATA2NORM_X
%
%    [ norm_x ] = data2norm( handle, data_x )
%
%    handle - handle to axes object
%    data_x - array with x-coordinates (as in data)
%
%    norm_x      - array with x-coordinates (normalized)
%--------------------------------------------------------------------------
function [ norm_x ] = data2norm_x ( handle, data_x )

position = get(handle, 'Position');
xlim = get(handle, 'XLim');

if strcmp(get(handle, 'XDir'), 'reverse')
    xlim = fliplr(xlim);
end

% Check whether the scale is logaritmic
if strcmp(get(handle, 'XScale'), 'log')
    negative_indices = find(data_x <= 0);
    for i = negative_indices
        data_x(i) = NaN;
    end
    xlim = log10(xlim);
    data_x = log10(data_x);
end
norm_x = position(1) + position(3) * (data_x-xlim(1))/(xlim(2)-xlim(1));


%- end of function data2norm_x --------------------------------------------



%% FUNCTION DATA2NORM_Y
%
%    [ norm_y ] = data2norm( handle, data_y )
%
%    handle - handle to axes object
%    data_y - array with data y-coordinates
%
%    norm_y      - array with normalized y-coordinates
%--------------------------------------------------------------------------
function [ norm_y ] = data2norm_y ( handle, data_y )

position = get(handle, 'Position');
ylim = get(handle, 'YLim');

if strcmp(get(handle, 'YDir'), 'reverse')
    ylim = fliplr(ylim);
end

if strcmp(get(handle, 'YScale'), 'log')
    negative_indices = find(data_y <= 0);
    for i = negative_indices
        data_y(i) = NaN;
    end
    ylim = log10(ylim);
    data_y = log10(data_y);
end

norm_y = position(2) + position(4) * (data_y-ylim(1))/(ylim(2)-ylim(1));

%- end of function data2norm_y --------------------------------------------



%% FUNCTION GET_NORMALIZED_XTICK
%
%    [ normalized_xtick ] = get_normalized_xtick( handle )
%    [ normalized_xtick ] = get_normalized_xtick( handle, no_limits )
%    [ normalized_xtick ] = get_normalized_xtick( handle, no_limits, includeMinors )
%
%    handle        - handle to the axes object
%    no_limits     - when this value is true, the xticks that are equal to
%                    xlim are not returned. (The first and last value in
%                    most situations). (default: false)
%    includeMinors - If true and the 'XMinorTick' propery is 'on', then
%                    minor ticks are included in the returned array. If
%                    false and/or the 'XMinorTick' property is 'off' the
%                    minor tick are not returned. (default: false)
% 
%    normalized_xtick - xtick property, but now normalized to figure
%--------------------------------------------------------------------------
function [ normalized_xtick ] = get_normalized_xtick( handle, varargin )

% no_limits
if nargin > 1
    no_limits = cell2mat(varargin(1));
else
    no_limits = false;
end

% includeMinors
if nargin > 2
    includeMinors = cell2mat(varargin(2));
else
    includeMinors = false;
end

xtick = get(handle, 'XTick');
xlim = get(handle, 'XLim');

if includeMinors
    if strcmp(get(handle, 'XMinorTick'), 'on') && strcmp(get(handle, 'XScale'), 'log') && ...
            (length(xtick) > 1)
        tickDiff = mean(diff(log10(xtick)));
        if tickDiff == 1
            current_tick = min(xlim);
            step = 10^floor(log10(current_tick));
            current_tick = ceil(current_tick/step)*step;
            while current_tick < max(xlim),
                xtick = [xtick current_tick];
                step = 10^floor(log10(current_tick));
                current_tick = current_tick + step;
            end
            xtick = unique(xtick);
        end
    end
end

% Remove ticks that are outside xlim (or equal to xlim if no_limits)
if no_limits
    xtick(xtick >= max(xlim) | xtick <= min(xlim)) = [];
else
    xtick(xtick > max(xlim) | xtick < min(xlim)) = [];
end

normalized_xtick = data2norm_x(handle, xtick);

%- end of function get_normalized_xtick -----------------------------------



%% FUNCTION GET_NORMALIZED_YTICK
%
%    [ normalized_ytick ] = get_normalized_ytick( handle )
%    [ normalized_ytick ] = get_normalized_ytick( handle, no_limits )
%    [ normalized_ytick ] = get_normalized_ytick( handle, no_limits, includeMinors )
% 
%    handle        - handle to the axes object
%    no_limits     - when this value is true, the xticks that are equal to
%                    xlim are not returned. (The first and last value in
%                    most situations). (default: false)
%    includeMinors - If true and the 'YMinorTick' propery is 'on', then
%                    minor ticks are included in the returned array. If
%                    false and/or the 'YMinorTick' property is 'off' the
%                    minor tick are not returned. (default: false)
%
%    normalized_ytick - ytick property, but now normalized to figure
%--------------------------------------------------------------------------
function [ normalized_ytick ] = get_normalized_ytick( handle, varargin )

if nargin > 1
    no_limits = cell2mat(varargin(1));
else
    no_limits = false;
end

% includeMinors
if nargin > 2
    includeMinors = cell2mat(varargin(2));
else
    includeMinors = false;
end

ytick = get(handle, 'YTick');
ylim = get(handle, 'YLim');

if includeMinors
    if strcmp(get(handle, 'YMinorTick'), 'on') && strcmp(get(handle, 'YScale'), 'log') && ...
            (length(ytick) > 1)
        tickDiff = mean(diff(log10(ytick)));
        if tickDiff == 1
            % Because Matlab does not provide information about how many minor
            % tick are drawn, we need to guess.
            current_tick = min(ylim);
            step = 10^floor(log10(current_tick));
            current_tick = ceil(current_tick/step)*step;
            while current_tick < max(ylim),
                ytick = [ytick current_tick];
                step = 10^floor(log10(current_tick));
                current_tick = current_tick + step;
            end
            ytick = unique(ytick);
        end
    end
end

% Remove ticks that are outside ylim (or equal to ylim if no_limits)
if no_limits
    ytick(ytick >= max(ylim) | ytick <= min(ylim)) = [];
else
    ytick(ytick > max(ylim) | ytick < min(ylim)) = [];
end

normalized_ytick = data2norm_y(handle, ytick);

%- end of function get_normalized_ytick -----------------------------------



%% FUNCTION TRIM_WHITESPACE
%
%  Remove whitespace at start and end of string
%
%  [ trimmed ] = trim_whitespace( str )
%
%    str - string that needs to be trimmed
%
%    trimmed - str without whitespace at start and end
%--------------------------------------------------------------------------
function [ trimmed ] = trim_whitespace( str )
trimmed = '';
for i = 1:length(str)
    if ~isspace(str(i))
        trimmed = str(i:length(str));
        break;
    end
end
for i = length(trimmed):-1:1
    if ~isspace(trimmed(i))
        trimmed = trimmed(1:i);
        break;
    end
end

%- end of function trim_whitespace ----------------------------------------



%% FUNCTION SET_AND_REMEMBER
%
% First the old propertyValue is stored (together with handle and
% propertyName) is the global matfig2pgf_props. Then the property is set to
% the new value.
%
% set_and_remember( handle, propertyName, propertyValue )
%
%--------------------------------------------------------------------------
function set_and_remember( handle, propertyName, propertyValue )
global matfig2pgf_props;

prop.handle = handle';
prop.name = propertyName;
prop.value = get(handle, propertyName);

i = length(matfig2pgf_props)+1;
matfig2pgf_props{i} = prop;

set(handle, propertyName, propertyValue);

%- end of set_and_remember ---------------------------------------



%% FUNCTION RESTORE_REMEMBERED_PROPERTIES
%
% Restore the properties in the global matfig2pgf_props. This global was
% earlier filled by the set_and_remember function.
%--------------------------------------------------------------------------
function restore_remembered_properties
global matfig2pgf_props

% Restored all properties in reversed order
n = length(matfig2pgf_props);
for i = n:-1:1
    prop = matfig2pgf_props{i};
    set(prop.handle, prop.name, prop.value);
end

%- end of function restore_remembered_properties --------------------------



%% FUNCTION GET_PARENT_AXES
%
%  Gets the handle to the parent object and checks whether it is an axes
%  object. If it isn't, get it's parent and check again. This is repeated
%  until an axes object is found. The handle to this axes object is
%  returned.
%
%--------------------------------------------------------------------------
function axesHandle = get_parent_axes( handle )

axesHandle = get(handle, 'Parent');
while ~strcmp(get(axesHandle, 'Type'), 'axes')
    axesHandle = get(axesHandle, 'Parent');
end

%- end of function get_parent_axes ----------------------------------------



%% FUNCTION SNAP_AXES_BORDERS
%
% Check all borders of all axes objects and snap them to the values defined
% in the option if they are within range.
%
%--------------------------------------------------------------------------
function snap_axes_borders( handle )

global matfig2pgf_opt;

snapdistance = matfig2pgf_opt.snapdistance;

axeshandles = get_axes_or_legend_handles(handle, 'axes');
for h = axeshandles
    % Make sure that the units of the axes are set to normalized
    if ~strcmp('normalized', get(h, 'Units'))
        set(h, 'Units', 'normalized');
    end
    
    pos = get(h, 'Position');
    newpos = pos;
    
    % Snap vertical borders
    snapleft = matfig2pgf_opt.snapleft * matfig2pgf_opt.fontsize / 72 ...
        * 2.54 / matfig2pgf_opt.figwidth;
    snapright = matfig2pgf_opt.snapright * matfig2pgf_opt.fontsize / 72 ...
        * 2.54 / matfig2pgf_opt.figwidth;
    snapright = 1 - snapright;
    left = pos(1);
    right = pos(1)+pos(3);
    
    if abs(snapleft-left) <= snapdistance
        left = snapleft;
    end
    if abs(snapright-right) <= snapdistance
        right = snapright;
    end
    newpos(1) = left;
    newpos(3) = right-left;
    
    % Snap horizontal borders
    snaptop = matfig2pgf_opt.snaptop * matfig2pgf_opt.fontsize / 72 ...
        * 2.54 / matfig2pgf_opt.figheight;
    if ~isempty(get(get(h, 'Title'), 'String'))
        snaptop = snaptop + 1;
    end
    snaptop = 1 - snaptop;
    snapbottom = matfig2pgf_opt.snapbottom * matfig2pgf_opt.fontsize / 72 ...
        * 2.54 / matfig2pgf_opt.figheight;
    top = pos(2)+pos(4);
    bottom = pos(2);
    
    if abs(snaptop-top) <= snapdistance
        top = snaptop;
    end
    if abs(snapbottom-bottom) <= snapdistance
        bottom = snapbottom;
    end
    newpos(2) = bottom;
    newpos(4) = top-bottom;
    
    % Set new position
    if max(abs(pos-newpos)) > 0
        set_and_remember(h, 'Position', newpos)
    end

end  % end of loop through all axes objects

%- end of function snap_axes_borders --------------------------------------



%% FUNCTION GET_AXES_OR_LEGEND_HANDLES
%
% Returns an array with all axes handles or all legend handles. Internally
% Matlab indentifies both as axes object, it needs some effort to figure
% out which is which.
%
% handles = get_axes_or_legend_handles(parentHandle, type)
%
%  parentHandle - handle
%  type         - string, either 'axes' or 'legends'
%
%--------------------------------------------------------------------------
function handles = get_axes_or_legend_handles(parentHandle, type)

% First get all axes handles, this includes the legends
allAxes = transpose(findobj(parentHandle, 'Type', 'axes'));
allLegends = [];
for h = allAxes
    allLegends = [allLegends legend(h)];
end
if strcmp(type, 'axes')
    handles = setdiff(allAxes, allLegends);
elseif strcmp(type, 'legends')
    handles = allLegends;
else
    handles = [];
end

%- end of function get_axes_handles


%% FUNCTION GET_PARENT_AXES_HANDLE
%
% Look up the parent of a handle and if it is an 'axes' return it.
% Otherwise check its parent and see if it is an 'axes'. Repeat this until
% and 'axes' is found or until a 'figure' is found. In the last case it
% will return -1
function axesHandle = get_parent_axes_handle(handle)
while true
    handle = get(handle, 'Parent');
    parentType = get(handle, 'Type');
    if strcmp(parentType, 'axes')
        axesHandle = handle;
        return;
    elseif strcmp(parentType, 'figure')
        axesHandle = -1;
        return;
    end
end

%- end of function get_parent_axes_handle


%% FUNCTION SPECIFY_PGF_COLOR
%
% Write PGF code to a file pointer to set a line or fill color.
function specify_pgf_color(fp, indent, type, colorspec)
if strcmpi(type, 'line')
    colorname = 'matfig2pgf_linecolor';
    pgfcmd = 'pgfsetstrokecolor';
elseif strcmpi(type, 'fill')
    colorname = 'matfig2pgf_fillcolor';
    pgfcmd = 'pgfsetfillcolor';
else
    error('Invalid type specified in function specify_pgf_color');
end
fprintf(fp,'%s\\definecolor{%s}{rgb}{%.3f,%.3f,%.3f}\n',...
    indent, colorname, colorspec(1), colorspec(2), colorspec(3));
fprintf(fp,'%s\\%s{%s}\n', ...
    indent, pgfcmd, colorname);

%- end of function specify_pgf_color
