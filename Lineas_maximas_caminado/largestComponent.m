function BW = largestComponent(BW)

[h,w] = size(BW);

visited = false(h,w);

bestCount = 0;
bestPixels = [];

for i=1:h
    for j=1:w

        if BW(i,j)==1 && ~visited(i,j)

            queue = [i j];
            visited(i,j)=true;

            pixels = [];

            while ~isempty(queue)

                p = queue(1,:);
                queue(1,:) = [];

                pixels = [pixels;p];

                r = p(1);
                c = p(2);

                neighbors = [
                    r-1 c
                    r+1 c
                    r c-1
                    r c+1
                ];

                for k=1:4

                    rr = neighbors(k,1);
                    cc = neighbors(k,2);

                    if rr>=1 && rr<=h && cc>=1 && cc<=w

                        if BW(rr,cc)==1 && ~visited(rr,cc)

                            visited(rr,cc)=true;
                            queue=[queue;rr cc];

                        end

                    end

                end

            end

            if size(pixels,1)>bestCount

                bestCount=size(pixels,1);
                bestPixels=pixels;

            end

        end

    end

end

BW(:)=0;

for k=1:size(bestPixels,1)

    BW(bestPixels(k,1),bestPixels(k,2))=1;

end