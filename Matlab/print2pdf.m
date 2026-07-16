function print2pdf(outfilename, doprint)

    if exist('doprint','var') && doprint

        fig = gcf;

        % Base output directory
        outdir = './Results/';

        % Subfolders
        figdir = fullfile(outdir, 'FIGS');
        pdfdir = fullfile(outdir, 'PDF');

        % Create folders if they do not exist
        if ~exist(figdir, 'dir')
            mkdir(figdir);
        end

        if ~exist(pdfdir, 'dir')
            mkdir(pdfdir);
        end

        % Make figure large on screen
        set(fig, 'Units', 'centimeters');
        set(fig, 'Position', [1 1 30 20]);

        % Save MATLAB figure
        savefig(fig, fullfile(figdir, append(outfilename,'.fig')));

        % Export PDF
        exportgraphics(fig, ...
            fullfile(pdfdir, append(outfilename,'.pdf')), ...
            'ContentType', 'vector', ...
            'Resolution', 600);

    end

end