function handle = drawRod(y, theta, L, gap, height, handle, mode)
X = [y, y+L*sin(theta)];
Y = [gap+height, gap + height + L*cos(theta)];
if isempty(handle)
handle = plot(X, Y, 'g', 'EraseMode', mode);
else
set(handle,'XData',X,'YData',Y);
end
