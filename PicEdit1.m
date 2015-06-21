clc;
clear all;

I = imread('lnd800x600.bmp');

[IY, IX, IZ] = size(I);

XO = 800;
YO = 600;
I = (I(:,:,1)+I(:,:,2)+I(:,:,3))*0.4;
I  = dither(I);
%I = (ImDith1b(I, IX, IY, 0, 0.3, 0));

%  for Y = 1:IY
%      for X = 1:IX
%          if(I(Y,X)>50)
%              I(Y,X) = 1;
%          else
%              I(Y,X) = 0;
%          end
%      end
%  end
PO = uint32(imresize(I, [YO, XO]));
ID(:,:,1) = PO;
ID(:,:,2) = PO;
ID(:,:,3) = PO;
ID = uint8(ID*255);

image(ID)

% for N = 1:16:XO*YO
%     PX = 0;
%     for C = 0:15
%         PX = PX + PO(ceil(N/XO)+C, ceil(mod(N, XO)))*2^(C);
%     end
%     ImO(ceil(1+N/16)) = PX;
% end

N = 1;
ImO = zeros(1, YO*XO/16);
for Y = 1:YO
    for X = 1:16:XO
        PX = 0;
        for C = 0:15
            PX = PX + bitshift(PO(Y, X+C), 15-C);
        end
        ImO(N) = PX;
        N = N+1;
    end
end

F = fopen('ImO.h', 'w');

fprintf(F, '#ifndef IMO_H\n#define IMO_H\n\n');

fprintf(F, 'const uint16_t Image[%d] = {\r\n', numel(ImO));

Lines = 15;
for N = 1:numel(ImO)
    fprintf(F, '%5.0f, ', ImO(N));
    if(mod(N, Lines) == Lines-1)
        fprintf(F, '\r\n');
    end
end

fprintf(F, '};\n\r#endif');

fclose(F);

% for Y = 1:IY
%     for X = 1:IX
%       R = uint8(I(Y, X, 1)/2^5);
%       G = uint8(I(Y, X, 2)/2^5);
%       B = uint8(I(Y, X, 3)/2^6);
%       PO(Y, X) = (R+(G*2^3)+(B*2^6));
%     end
% end

% fprintf('const uint8_t Ar[%.0f] = {\n', XO*YO);
% for Y = 1:YO
%     for X = 1:XO
%       fprintf('%3.0f, ', PO(Y, X));
%       if(mod(X, 16) == 15)
%           fprintf('\n');
%       end
%     end
% end
% fprintf('};\n\n')