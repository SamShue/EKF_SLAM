clc
%structure;
structure.a=1;
structure.b=2;
structure=test(structure);
structure.b

function value=test(structure)
    structure.a=structure.a+1;
    structure.b=structure.b+2;
    value=structure;
end 