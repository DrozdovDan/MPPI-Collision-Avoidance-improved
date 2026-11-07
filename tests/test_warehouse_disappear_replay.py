"""Recovery must preserve every complete frame of an interrupted replay."""
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'benchmarks'))
from multi_agent.warehouse_replay import StreamingGif,_palette
from multi_agent.warehouse_disappear_replay import resume_writer


class RecoveryTests(unittest.TestCase):
    def create(self,folder):
        path=Path(folder)/'test.gif'
        (Path(folder)/'replay_renderer.initial.py').write_text('# Synthetic original renderer\n')
        palette=_palette([(1.,0.,0.),(0.,0.,1.),(0.,1.,0.)])
        frames=[Image.new('RGB',(8,8),'red') for _ in range(3)]
        frames[1].paste('blue',(0,0,4,4));frames[2].paste('green',(2,2,8,8))
        writer=StreamingGif(path,palette)
        for frame in frames:writer.append(frame,100)
        writer.close()
        with Image.open(path) as gif:
            expected=[]
            for i in range(gif.n_frames):gif.seek(i);expected.append(np.array(gif.convert('RGB')))
        return path,palette,frames,expected

    def test_resume_truncated_final_image_preserves_complete_prefix(self):
        with tempfile.TemporaryDirectory() as folder:
            path,palette,frames,expected=self.create(folder)
            path.write_bytes(path.read_bytes()[:-3])
            writer,count=resume_writer(path,palette)
            self.assertEqual(count,2)
            prefix=path.read_bytes()
            writer.append(frames[2],100);writer.close()
            self.assertTrue(path.read_bytes().startswith(prefix))
            with Image.open(path) as gif:
                self.assertEqual(gif.n_frames,3)
                for i in range(3):
                    gif.seek(i)
                    np.testing.assert_array_equal(np.array(gif.convert('RGB')),expected[i])

    def test_complete_gif_resume_without_new_frames_is_byte_identical(self):
        with tempfile.TemporaryDirectory() as folder:
            path,palette,_,_=self.create(folder);original=path.read_bytes()
            writer,count=resume_writer(path,palette);writer.close()
            self.assertEqual(count,3)
            self.assertEqual(path.read_bytes(),original)


if __name__=='__main__':unittest.main()
