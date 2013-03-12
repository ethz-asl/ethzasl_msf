function varargin = matfig2pgf_gui_options( cmd, varargin )
% MATFIG2PGF_GUI_OPTIONS  Display a windows with the Matfig2PGF options.
%
%   matfig2pgf_gui_options(<cmd>)
%   matfig2pgf_gui_options(<cmd>, <options_struct>)
%   matfig2pgf_gui_options(<cmd>, <option>, <value>, ...)
%
%   Example:
%     matfig2pgf_gui_options('resize', 'fignr', 2)
%
%   Commands:
%     'resize'     - Only resize the figure.
%     'save'       - Only save the figure.
%     'resizesave' - Resize and save the figure.


%% --- INITIALIZATION TASKS
switch(cmd)
    case 'resize'
        fh = create_resize_options_window();
    case 'save'
        fh = create_save_options_window();
    case 'resizesave'
        fh = create_resizesave_options_window();
    otherwise
        error('matfig2pgf_gui_options:UnknownCommand', 'Unknown command: %s', cmd);
end

len = length(varargin);
varargin{len+1} = 'guioptions';
varargin{len+2} = true;
opt = matfig2pgf_options('get_options', varargin);

copy_options_to_gui(opt, fh);

set(fh, 'Visible', 'on');

%% --- CALLBACKS
%% Function callback_Ok_Button
    function callback_Ok_Button(hObject, eventdata, cmd)
        opt = get_options_from_gui(fh);
        close(fh);
        
        % Store the options chosen in the gui in a global
        global matfig2pgf_opt_gui;
        matfig2pgf_opt_gui = opt;
        
        % Resize the figure
        switch cmd
            case 'resize'
                opt.noresize = false;
                opt.nosave = true;
            case 'save'
                opt.noresize = true;
                opt.nosave = false;
            case 'resizesave'
                opt.noresize = false;
                opt.nosave = false;
        end
        opt.guioptions = true;
        matfig2pgf(opt);
    end



%% Function callback_Cancel_Button
    function callback_Cancel_Button(hObject, eventdata)
        close(fh);
    end



%% Function callback_Reset_Button
    function callback_Reset_Button(hObject, eventdata)
        clear global matfig2pgf_opt_gui;
        opt = matfig2pgf_options('get_options');
        copy_options_to_gui(opt, fh);
    end


%% --- UTILITY FUNCTIONS
%% Function copy_options_to_gui
    function copy_options_to_gui(opt, fh)
        handles = guihandles(fh);
        fields = fieldnames(opt);
        for i = 1:length(fields)
            fieldname = fields{i};
            if ~isfield(handles, fieldname)
                continue;
            end
            
            style = get(handles.(fieldname), 'Style');
            switch style
                case 'edit'
                    fieldvalue = matfig2pgf_options('convert_to_char', opt.(fieldname));
                    set(handles.(fieldname), 'String', fieldvalue);
                case 'checkbox'
                    fieldvalue = matfig2pgf_options('convert_to_bool', opt.(fieldname));
                    set(handles.(fieldname), 'Value', fieldvalue);
                case 'popupmenu'
                    names = get(handles.(fieldname), 'String');
                    for j = 1:length(names)
                        if strcmpi(names{j}, opt.(fieldname))
                            set(handles.(fieldname), 'Value', j);
                            break;
                        end
                    end
            end
        end
    end



%% Function get_options_from_gui
    function opt = get_options_from_gui(fh)
        field_info = matfig2pgf_options('get_field_info');
        handles = guihandles(fh);
        for i = 1:length(field_info)
            fieldname = field_info(i).name;
            if ~isfield(handles, fieldname)
                continue;
            end
            
            style = get(handles.(fieldname), 'Style');
            switch style
                case 'checkbox'
                    opt.(fieldname) = get(handles.(fieldname), 'Value');
                case 'edit'
                    opt.(fieldname) = get(handles.(fieldname), 'String');
                case 'popupmenu'
                    index = get(handles.(fieldname), 'Value');
                    strings = get(handles.(fieldname), 'String');
                    opt.(fieldname) = strings{index};
            end
        end
    end

    

%% Function create_resize_options_window
    function fh = create_resize_options_window()
        fh = figure('Name', 'Matfig2PGF resize options', ...
            'NumberTitle', 'off', 'MenuBar', 'none', 'Resize', 'off', ...
            'Color', get(0,'defaultUicontrolBackgroundColor'), ...
            'Units', 'normalized', 'Position', [0 0 1 1], ...
            'Units', 'characters', 'Visible', 'off');

        width = 74;
        height = 20;
        pos = get(fh, 'Position');
        set(fh, 'Position', [0.5*(pos(3)-width) 0.5*(pos(4)-height) width height]);

        ph = create_resize_options_controls(fh);
        pos = get(ph, 'Position');
        set(ph, 'Position', [2 4 pos(3) pos(4)]);

        uicontrol(fh, 'Style', 'pushbutton', 'String', 'Resize figure', ...
            'Units', 'characters', 'Position', [2 1 22 2], ...
            'Callback', {@callback_Ok_Button, 'resize'});
        uicontrol(fh, 'Style', 'pushbutton', 'String', 'Cancel', ...
            'Units', 'characters', 'Position', [26 1 22 2], ...
            'Callback', @callback_Cancel_Button);
        uicontrol(fh, 'Style', 'pushbutton', 'String', 'Reset to default', ...
            'Units', 'characters', 'Position', [50 1 22 2], ...
            'Callback', @callback_Reset_Button);
    end



%% Function create_resize_options_controls
    function ph = create_resize_options_controls(fh)
        ph = uipanel(fh, 'Title', 'Resize options', ...
            'Units', 'characters', 'Position', [0 0 70 15]);
        
        %
        % Figure appearance panel
        %
        uicontrol(ph, 'Style', 'text', 'String', 'Figure width ', ...
            'Units', 'characters', 'Position', [2.1 12 14.9 1.2], ...
            'HorizontalAlignment', 'right');
        uicontrol(ph, 'Tag', 'figwidth', 'Style', 'edit', ...
            'BackgroundColor', 'white', ...
            'Units', 'characters', 'Position', [17.1 12 6.4 1.5]);
        uicontrol(ph, 'Style', 'text', 'String', 'cm', ...
            'Units', 'characters', 'Position', [23.5 12 4.3 1.2]);

        uicontrol(ph, 'Style', 'text', 'String', 'Font ', ...
            'HorizontalAlignment', 'right', ...
            'Units', 'characters', 'Position', [2.1 10 14.9 1.2]);
        uicontrol(ph, 'Tag', 'fontname', 'Style', 'popupmenu', ...
            'String', {'AvantGarde', 'Bookman', 'Courier', 'Helvetica', ...
            'NewCenturySchoolbook', 'Palatino', 'Times'}, ...
            'Units', 'characters', 'Position', [17.1 10 27.8 1.5], ...
            'BackgroundColor', 'white');

        uicontrol(ph, 'Style', 'text', 'String', 'Size ', ...
            'Units', 'characters', 'Position', [46.9 10 6.4 1.2], ...
            'HorizontalAlignment', 'right');
        uicontrol(ph, 'Tag', 'fontsize', 'Style', 'edit', ...
            'BackgroundColor', 'white', ...
            'Units', 'characters', 'Position', [53.5 10 6.4 1.5]);
        uicontrol(ph, 'Style', 'text', 'String', 'pt', ...
            'Units', 'characters', 'Position', [60.5 10 4.3 1.2], ...
            'HorizontalAlignment', 'left');


        %
        % Snap axes objects panel
        %
        uicontrol(ph, 'Style', 'text', 'String', 'Snap axes objects', ...
            'Units', 'characters', 'Position', [2 7 50 1.5], ...
            'HorizontalAlignment', 'left', ...
            'FontSize', 1.25*get(0, 'DefaultUicontrolFontSize'), 'FontWeight', 'bold');
        
        uicontrol(ph, 'Style', 'text', 'Units', 'characters', 'String', 'Snap distance ', ...
            'Position', [2.1 5 14.9 1.2], 'HorizontalAlignment', 'right');
        uicontrol(ph, 'Tag', 'snapdistance', 'Style', 'edit', 'Units', 'characters', ...
            'Position', [17.1 5 6.4 1.5], 'BackgroundColor', 'white');

        uicontrol(ph, 'Style', 'text', 'String', 'Left ', ...
            'Units', 'characters', 'Position', [2.1 3 14.9 1.2], 'HorizontalAlignment', 'right');
        uicontrol(ph, 'Tag', 'snapleft', 'Style', 'edit', 'BackgroundColor', 'white', ...
            'Units', 'characters', 'Position', [17.1 3 6.4 1.5]);

        uicontrol(ph, 'Style', 'text', 'String', 'Right ', ...
            'Units', 'characters', 'Position', [25.6 3 8.5 1.2], 'HorizontalAlignment', 'right');
        uicontrol(ph, 'Tag', 'snapright', 'Style', 'edit', 'BackgroundColor', 'white', ...
            'Units', 'characters', 'Position', [34 3 6.4 1.5]);

        uicontrol(ph, 'Style', 'text', 'String', 'Top ', ...
            'Units', 'characters', 'Position', [2.1 1 14.9 1.2], 'HorizontalAlignment', 'right');
        uicontrol(ph, 'Tag', 'snaptop', 'Style', 'edit', 'BackgroundColor', 'white', ...
            'Units', 'characters', 'Position', [17.1 1 6.4 1.5]);

        uicontrol(ph, 'Style', 'text', 'String', 'Bottom ', ...
            'Units', 'characters', 'Position', [25.6 1 8.5 1.2], 'HorizontalAlignment', 'right');
        uicontrol(ph, 'Tag', 'snapbottom', 'Style', 'edit', 'BackgroundColor', 'white', ...
            'Units', 'characters', 'Position', [34 1 6.4 1.5]);
    end



%% Function create_save_options_window
    function fh = create_save_options_window()
        fh = figure('Name', 'Matfig2PGF save options', ...
            'NumberTitle', 'off', 'MenuBar', 'none', 'Resize', 'off', ...
            'Color', get(0,'defaultUicontrolBackgroundColor'), ...
            'Units', 'normalized', 'Position', [0 0 1 1], ...
            'Units', 'characters', 'Visible', 'off');

        width = 74;
        height = 24;
        pos = get(fh, 'Position');
        set(fh, 'Position', [0.5*(pos(3)-width) 0.5*(pos(4)-height) width height]);

        ph = create_save_options_controls(fh);
        pos = get(ph, 'Position');
        set(ph, 'Position', [2 4 pos(3) pos(4)]);

        uicontrol(fh, 'Style', 'pushbutton', 'String', 'Save as pgf', ...
            'Units', 'characters', 'Position', [2 1 22 2], ...
            'Callback', {@callback_Ok_Button, 'save'});
        uicontrol(fh, 'Style', 'pushbutton', 'String', 'Cancel', ...
            'Units', 'characters', 'Position', [26 1 22 2], ...
            'Callback', @callback_Cancel_Button);
        uicontrol(fh, 'Style', 'pushbutton', 'String', 'Reset to default', ...
            'Units', 'characters', 'Position', [50 1 22 2], ...
            'Callback', @callback_Reset_Button);
    end



%% Function create_save_options_controls
    function ph = create_save_options_controls(fh)
        ph = uipanel(fh, 'Title', 'Save options', 'Units', 'characters', ...
            'Position', [0 0 70 19]);
        
        uicontrol(ph, 'Style', 'text', 'String', 'Minimal line width', ...
            'HorizontalAlignment', 'left', ...
            'Units', 'Characters', 'Position', [2 16 16.5 1.2]);
        uicontrol(ph, 'Tag', 'minlinewidth', 'Style', 'edit', ...
            'BackgroundColor', 'white', ...
            'Units', 'Characters', 'Position', [19.5 16 6 1.5]);
        uicontrol(ph, 'Style', 'text', 'String', 'pt', ...
            'HorizontalAlignment', 'left', ...
            'Units', 'Characters', 'Position', [26.5 16 6 1.2]);
        
        uicontrol(ph, 'Style', 'text', 'String', 'Reduced line error', ...
            'HorizontalAlignment', 'left', ...
            'Units', 'Characters', 'Position', [2 14 18 1.2]);
        uicontrol(ph, 'Tag', 'reducedlineerror', 'Style', 'edit', ...
            'BackgroundColor', 'white', ...
            'Units', 'Characters', 'Position', [21 14 6 1.5]);
        
        uicontrol(ph, 'Tag', 'drawfigoutline', 'Style', 'checkbox', ...
            'String', 'Draw box around figure', ...
            'Units', 'Characters', 'Position', [2 12 30 1.5]);
        
        uicontrol(ph, 'Tag', 'converttexttolatex', 'Style', 'checkbox', ...
            'String', 'Convert text to LaTeX', ...
            'Units', 'Characters', 'Position', [2 9 25 1.5]);
        
        uicontrol(ph, 'Style', 'text', 'String', 'LaTeX text:  code before', ...
            'HorizontalAlignment', 'left', ...
            'Units', 'Characters', 'Position', [2 7 25 1.2]);
        uicontrol(ph, 'Tag', 'textprecode', 'Style', 'edit', ...
            'BackgroundColor', 'white', 'HorizontalAlignment', 'left', ...
            'Fontname', get(0,'FixedWidthFontName'), ...
            'Units', 'Characters', 'Position', [28 7 39 1.5]);
        
        uicontrol(ph, 'Style', 'text', 'String', 'after', ...
            'HorizontalAlignment', 'right', ...
            'Units', 'Characters', 'Position', [2 5 25 1.2]);
        uicontrol(ph, 'Tag', 'textpostcode', 'Style', 'edit', ...
            'BackgroundColor', 'white', 'HorizontalAlignment', 'left', ...
            'Fontname', get(0,'FixedWidthFontName'), ...
            'Units', 'Characters', 'Position', [28 5 39 1.5]);

        uicontrol(ph, 'Style', 'text', 'String', 'LaTeX math:  code before', ...
            'HorizontalAlignment', 'left', ...
            'Units', 'Characters', 'Position', [2 3 25 1.2]);
        uicontrol(ph, 'Tag', 'mathprecode', 'Style', 'edit', ...
            'BackgroundColor', 'white', 'HorizontalAlignment', 'left', ...
            'Fontname', get(0,'FixedWidthFontName'), ...
            'Units', 'Characters', 'Position', [28 3 39 1.5]);
        
        uicontrol(ph, 'Style', 'text', 'String', 'after', ...
            'HorizontalAlignment', 'right', ...
            'Units', 'Characters', 'Position', [2 1 25 1.2]);
        uicontrol(ph, 'Tag', 'mathpostcode', 'Style', 'edit', ...
            'BackgroundColor', 'white', 'HorizontalAlignment', 'left', ...
            'Fontname', get(0,'FixedWidthFontName'), ...
            'Units', 'Characters', 'Position', [28 1 39 1.5]);

    end



%% Function create_resizesave_options_window
    function fh = create_resizesave_options_window()
        fh = figure('Name', 'Matfig2PGF options', ...
            'NumberTitle', 'off', 'MenuBar', 'none', 'Resize', 'off', ...
            'Color', get(0,'defaultUicontrolBackgroundColor'), ...
            'Units', 'normalized', 'Position', [0 0 1 1], ...
            'Units', 'characters', 'Visible', 'off');

        width = 150;
        height = 21;
        pos = get(fh, 'Position');
        set(fh, 'Position', [0.5*(pos(3)-width) 0.5*(pos(4)-height) width height]);

        ph = create_resize_options_controls(fh);
        pos = get(ph, 'Position');
        set(ph, 'Position', [2 5 pos(3) pos(4)]);
        
        ph = create_save_options_controls(fh);
        pos = get(ph, 'Position');
        set(ph, 'Position', [75 1 pos(3) pos(4)]);

        uicontrol(fh, 'Style', 'pushbutton', 'String', 'Resize & save', ...
            'Units', 'characters', 'Position', [2 1 22 2], ...
            'Callback', {@callback_Ok_Button, 'resizesave'});
        uicontrol(fh, 'Style', 'pushbutton', 'String', 'Cancel', ...
            'Units', 'characters', 'Position', [26 1 22 2], ...
            'Callback', @callback_Cancel_Button);
        uicontrol(fh, 'Style', 'pushbutton', 'String', 'Reset to default', ...
            'Units', 'characters', 'Position', [50 1 22 2], ...
            'Callback', @callback_Reset_Button);
    end


end
