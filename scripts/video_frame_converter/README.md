##extractor.bat

Extracts all frames from video files.

###Install

- Download FFMPEG (https://ffmpeg.org/download.html)
- Extract /bin/ffmpeg.exe (you don't need the rest) to the same folder as extractor.bat

###Run

- Drag & drop video files on extractor.bat file.
    - The script will create a folder named after the video.
    - The folder will be in the same location as the video.
    - The images will be extracted to that folder.

###Config

- The script can not be configured, so it must be edited instead.
- Look for the line that contains "ffmpeg".
    - -r: framerate
        - 60/1 -> 1 second of video produces 60 frames.
    - %%04d: How the frames will be named -> 4 numbers with leading zeroes.


##images_to_video.bat

Creates a video from multiple images.

###Install

Same is extractor.bat

###Run

- Copy or move images to the same folder as images_to_video.bat
- Run images_to_video.bat by double-clicking on it (via File Explorer).
- Remember to delete the images when done.

###Config

- The script can not be configured, so it must be edited instead.
    - -r: framerate
        - 24 -> 24 images produce 1 second of video.
    - -start_number: the image to start with. All images before will be ignored.
    - -i: input files
    - -c:v libx264 -> the codec to use to compress the video.
    - -pix_fmt yuv420p -> needed for libx264 codec, otherwise video will be corrupt.
