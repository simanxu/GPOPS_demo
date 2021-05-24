function links = Slide_link(sollink)

left_phase = sollink.left.phase;
right_phase = sollink.right.phase;

xf_left = sollink.left.state;
p_left = sollink.left.parameter;

x0_right = sollink.right.state;
p_right = sollink.right.parameter;

links = x0_right - xf_left;

end


