function col=getColorFromID(id)
	% get rgb [0 1] values from id
% 

    if id==0,  col=zeros(1, 3); return; end
    
    colors=getIDColors;
    col=colors((mod(id, size(colors, 1)))+1, :);
end

function colors=getIDColors()
colors=[
      128 255 255;    % 
      255 0 0;        % red           1
      0  255  0;        % green         2
      0  0  255;        % blue          3
      0  255  255;      % cyan          4
      255  0  255;      % magenta       5
      212  212  0;      % yellow        6
      25  25  25;       % black         7
      34 139 34;      % forestgreen   8
      0 191 255;      % deepskyblue   9
      139 0 0 ;       % darkred       10
      218 112 214;    % orchid        11
      244 164 96; % sandybrown    12
      245 245 245; % white smoke      13
      139 119 101;    % peach puff 4         14
      105 105 105;    % dim gray     15
      25 25 112;      % midnight blue 16
      70 130 180;     % steel blue   17
      64 224 208;     % turqoise     18
      95 158 160;     % cadet blue   19
      106 19 205;     % slate blue   20
      102 205 170;     % Medium Aquamarine       21
      152 251 152;     % Pale Green   22
      240 230 140;     % Khaki   23
      255 215 0;     % Gold   24
      184 134 11;     % Dark Goldenrod   25      
      188 143 143;     % Rosy Brown   26
      160 82 45;     % Sienna   27
      245 245 220;     % Beige   28
      210 180 140;     %  Tan  29
      178 34 34;     % Firebrick   30
      233 150 122;     % Dark Salmon       31
      255 165 0;     %  Orange  32
      255 99 71;     % Tomato   33
      240 128 128;     % Light Coral       34
      255 105 180;     % Hot Pink         35      
      255 192 203;     %  Pink  36
      221 160 221;     % Plum   37
      245 222 179;     % Wheat     38
      255 250 205;     %  Lemon Chiffon   39
      160 32 240;     %  Purple  40
      210 105 30;     %  Chocolate  41
      127 255 0;     %  Chartreuse      42
      153 50 204;     %  Dark Orchid    43
      216 191 216;];     % Thistle   44      
      
      
      
colors = colors / 255;

end