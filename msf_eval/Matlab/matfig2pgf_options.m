function varargout = matfig2pgf_options(cmd, varargin)
% MATFIG2PGF_OPTIONS  Manage Matfig2PGF options. Used by Matfig2PGF.
%
%   matfig2pgf_options(<cmd>)
%   matfig2pgf_options(<cmd>, <options_struct>)
%   matfig2pgf_options(<cmd>, <option>, <value>, ...)
%
%   Example:
%     matfig2pgf_options('set_global', 'figwidth', 8)
%
%   Commands:
%     'set_global'      - retrieves the options and stores them in the
%                         global variable matfig2pgf_opt. Options are
%                         retrieved in the following order: 1) options
%                         specified on command line (in options_struct or
%                         option/value pairs), 2) user defined default
%                         options, and 3) program default options.
%     'get_options'     - retrieves the options and returns them in a
%                         struct. The options are retrieved in the same way
%                         as for the 'set_global' command.
%     'convert_to_char' - convert a value to a valid string.
%     'convert_to_bool' - convert a value to a valid boolean.
%     'get_field_info'  - return the struct with information about all
%                         option fields.

% Unwrap the varargin cell array. If a cell varargin cell array is passed
% to another function with varargin argument is gets wrapped in another
% cell array.
while (length(varargin) == 1) && iscell(varargin{1})
    varargin = varargin{1};
end

switch cmd
    case 'set_global'
        set_global(varargin);
        varargout = {};
    case 'get_options'
        varargout{1} = get_options(varargin);
    case 'convert_to_char'
        varargout{1} = convert_to_char(varargin{1});
    case 'convert_to_bool'
        varargout{1} = convert_to_boolean(varargin{1});
    case 'get_field_info'
        varargout{1} = get_field_info();
    otherwise
        varargout = {};
end

%- end of function matfig2pgf_options -------------------------------------



%% FUNCTION GET_FIELD_INFO
%
% Gets structure with possible options and information about these options.
%
%--------------------------------------------------------------------------
function defs = get_field_info()
defs = [ ...
    struct('name', 'fignr',          'type', 'num',  'default', 0) ...
    struct('name', 'filename',       'type', 'char', 'default', '') ...
    struct('name', 'figwidth',       'type', 'num',  'default', 10) ...
    struct('name', 'fontname',       'type', 'char', 'default', 'Times') ...
    struct('name', 'fontsize',       'type', 'num',  'default', 10) ...
    struct('name', 'texticklabelsize','type', 'char', 'default', 'normalsize') ...
    struct('name', 'textextlabelsize','type', 'char', 'default', 'normalsize') ...
    struct('name', 'drawfigoutline', 'type', 'bool', 'default', false) ...
    struct('name', 'reducedlineerror', 'type', 'num', 'default', 0.1) ...
    struct('name', 'minlinewidth',   'type', 'num',  'default', 0.5) ...
    struct('name', 'includedraftcode', 'type', 'bool',  'default', true) ...
    struct('name', 'textprecode',    'type', 'char', 'default', '') ...
    struct('name', 'textpostcode',   'type', 'char', 'default', '') ...
    struct('name', 'mathprecode',    'type', 'char', 'default', '$') ...
    struct('name', 'mathpostcode',   'type', 'char', 'default', '$') ...
    struct('name', 'snapdistance',   'type', 'num',  'default', 0.1) ...
    struct('name', 'snapleft',       'type', 'num',  'default', 4) ...
    struct('name', 'snapright',      'type', 'num',  'default', 0.8) ...
    struct('name', 'snapbottom',     'type', 'num',  'default', 3) ...
    struct('name', 'snaptop',        'type', 'num',  'default', 0.8) ...
    struct('name', 'converttexttolatex', 'type', 'bool', 'default', true) ...
    struct('name', 'keepresizedfigure', 'type', 'bool', 'default', false) ...
    struct('name', 'noresize',       'type', 'bool', 'default', false) ...
    struct('name', 'nosave',         'type', 'bool', 'default', false) ...
    struct('name', 'guioptions',     'type', 'bool', 'default', false) ...
    ];
%- end of function get_field_defs -----------------------------------------



%% FUNCTION SET_GLOBAL
%
% Retrieves the options and stores them in the global matfig2pgf_opt
%
%--------------------------------------------------------------------------
function set_global(args)
global matfig2pgf_opt;
matfig2pgf_opt = get_options(args);
%- end of function set_global ---------------------------------------------



%% FUNCTION GET_OPTIONS
%
% Retrieve the options and returns a struct with all options
%
%--------------------------------------------------------------------------
function opt = get_options(args)

opt = get_default_program_options();
opt = combine_structs(opt, get_default_user_options());

opt_args = process_option_arguments(args);

global matfig2pgf_opt_gui;
if isstruct(matfig2pgf_opt_gui) && ...
        isfield(opt_args, 'guioptions') && opt_args.guioptions
    opt = combine_structs(opt, matfig2pgf_opt_gui);
end

opt = combine_structs(opt, opt_args);

%- end of function get_options --------------------------------------------



%% FUNCTION PROCESS_OPTION_ARGUMENTS
%
%  Converts the arguments to an options struct.
%
%--------------------------------------------------------------------------
function opt = process_option_arguments(args)

opt = struct();
if (length(args) == 1) && isstruct(args{1})
    opt = args{1};
elseif (length(args) == 1) && ischar(args{1})
    opt.filename = args{1};
elseif length(args) > 1
    % Check number of arguments
    if rem(length(args), 2)
        error('matfig2pgf:invalidNumberOfArguments', ...
            'Invalid number of arguments. Does every option has a value?');
    end
    % Process the option arguments
    current_arg = 1;
    while current_arg < length(args)
        optionname = args{current_arg};
        optionvalue = args{current_arg+1};
        checked_value = check_name_and_value(optionname, optionvalue);
        if isempty(checked_value)
            error('matfig2pgf:invalidOption', ...
                [optionname ' is not a valid option.']);
        end
        opt.(optionname) = checked_value;
        current_arg = current_arg+2;
    end    
end

%- end of function process_option_arguments -------------------------------


%% FUNCTION GET_DEFAULT_PROGRAM_OPTIONS
%
%  Get struct array with default program options
%
%--------------------------------------------------------------------------
function opt = get_default_program_options()
fields = get_field_info();
for field = fields
    opt.(field.name) = field.default;
end
%- end of function get_default_program_options ----------------------------



%% FUNCTION GET_DEFAULT_USER_OPTIONS
%
%  Get struct array with default user defined options 
%
%--------------------------------------------------------------------------
function opt = get_default_user_options()
opt = struct();
path = fullfile(fileparts(mfilename('fullpath')), 'useroptions.cfg');
fid = fopen(path);
if fid == -1
    return
end

while (~feof(fid));
    line = strtrim(fgetl(fid));
    if regexp(line, '^[%#]')
        continue;
    end
    tokens = regexp(line, '^([^=\s]+)\s*=\s*(.*)', 'tokens');
    if isempty(tokens)
        continue;
    end
    fieldname = tokens{1}{1};
    fieldvalue = tokens{1}{2};
    value = check_name_and_value(fieldname, fieldvalue);
    if ~isempty(value)
        opt.(fieldname) = value;
    end
end
fclose(fid);
%- end of function get_default_user_options



%% FUNCTION CHECK_NAME_AND_VALUE
%
% Checks whether the supplied option name exists. If it is not a valid
% option name it returns an empty array. If the option name is valid is
% make sure that the option value has the correct type and returns the
% value.
%--------------------------------------------------------------------------
function ret = check_name_and_value(name, value)
ret = [];
for field = get_field_info()
    if strcmp(field.name, name)
        switch field.type
            case 'num'
                ret = convert_to_num(value);
                return;
            case 'char'
                ret = convert_to_char(value);
                return;
            case 'bool'
                ret = convert_to_boolean(value)';
                return;
        end
    end
end
%- end of function check_name_and_value -----------------------------------



%% FUNCTION CONVERT_TO_NUM
%
% Convert a value to a valid number
%
%--------------------------------------------------------------------------
function ret = convert_to_num(value)
if isnumeric(value)
    ret = value;
elseif ischar(value)
    ret = str2double(value);
elseif islogical(value)
    if value
        ret = 1;
    else
        ret = 0;
    end
end
%- end of function convert_to_num -----------------------------------------



%% FUNCTION CONVERT_TO_CHAR
%
% Convert a value to a valid string.
%
%--------------------------------------------------------------------------
function ret = convert_to_char(value)
if isnumeric(value)
    ret = num2str(value, 10);
elseif ischar(value)
    ret = value;
elseif islogical(value)
    if value
        ret = 'true';
    else
        ret = 'false';
    end
end
%- end of function convert_to_char ----------------------------------------



%% FUNCTION CONVERT_TO_BOOLEAN
%
% Convert a value to a valid boolean
%
%--------------------------------------------------------------------------
function bool = convert_to_boolean(value)
if isnumeric(value)
    bool = (value ~= 0);
elseif ischar(value)
    value = strtrim(value);
    if length(value) > 1
        bool = strcmpi(value, 'true');
    else
        bool = ~strcmp(value, '0');
    end
elseif islogical(value)
    bool = value;
end

%- end of function convert_to_boolean -------------------------------------



%% FUNCTION COMBINE_STRUCTS
%
% Combines values from two structs. If both structs have the same field,
% the field from struct two is used.
%
%--------------------------------------------------------------------------
function target = combine_structs ( one, two )
target = one;
names = fieldnames(two);
for i = 1:length(names)
    name = names{i};
    value = two.(name);
    value = check_name_and_value(name, value);
    if ~isempty(value)
        target.(name) = value;
    end
end
%- end of function copy_struct_to_struct ----------------------------------
