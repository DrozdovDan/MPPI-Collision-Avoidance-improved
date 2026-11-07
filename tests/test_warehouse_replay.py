"""A streamed full replay must retain all frames, palettes and frame durations."""
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'benchmarks'))
from multi_agent.warehouse_replay import StreamingGif, _palette


class StreamingGifTests(unittest.TestCase):
    def test_delta_frames_decode_to_complete_images_with_recorded_timing(self):
        palette = _palette([[1., 0., 0.], [0., 0., 1.]])
        images = []
        for x in (1, 4, 7):
            im = Image.new('RGB', (16, 10), 'white')
            ImageDraw.Draw(im).rectangle((x, 2, x+2, 4), fill='red')
            images.append(im)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'replay.gif'; writer = StreamingGif(path, palette)
            for im, duration in zip(images, (2000, 100, 5000)):
                writer.append(im, duration)
            writer.close()
            with Image.open(path) as gif:
                self.assertEqual(gif.n_frames, 3)
                self.assertEqual(gif.info['loop'], 0)
                for i, duration in enumerate((2000, 100, 5000)):
                    gif.seek(i)
                    self.assertEqual(gif.info['duration'], duration)
                    expected = images[i].quantize(palette=palette, dither=Image.Dither.NONE).convert('RGB')
                    np.testing.assert_array_equal(np.array(gif.convert('RGB')), np.array(expected))

    def test_identical_frames_are_retained(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'replay.gif'; writer = StreamingGif(path, _palette([[0., .5, 1.]]))
            frame = Image.new('RGB', (8, 8), 'white')
            writer.append(frame, 100); writer.append(frame, 100); writer.close()
            with Image.open(path) as gif:
                self.assertEqual(gif.n_frames, 2)
                first = np.array(gif.convert('RGB'))
                gif.seek(1)
                np.testing.assert_array_equal(np.array(gif.convert('RGB')), first)


if __name__ == '__main__':
    unittest.main()
