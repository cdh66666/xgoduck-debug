import importlib.util
from pathlib import Path
import unittest
import numpy as np

PATH=Path(__file__).resolve().parents[1]/'tools/build_inertial_candidate.py'
spec=importlib.util.spec_from_file_location('inertial_candidate',PATH)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class InertialCandidateTests(unittest.TestCase):
    def test_tetrahedron_integration_matches_a_translated_box(self):
        vertices=np.array([[x,y,z] for x in (-.02,.02) for y in (-.01,.01) for z in (-.03,.03)])
        faces=np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],
                        [2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]])
        center=np.array([-.0178,.001,-.0328]);triangles=vertices[faces]+center
        for t in (triangles,triangles[:,::-1]):
            volume,actual,inertia=module.properties(t)
            self.assertAlmostEqual(volume,.04*.02*.06)
            np.testing.assert_allclose(actual,center,atol=1e-12)
            expect=np.diag([(.02**2+.06**2)/12,(.04**2+.06**2)/12,(.04**2+.02**2)/12])
            np.testing.assert_allclose(inertia,expect,atol=1e-12)

    def test_assembly_uses_parallel_axis_theorem(self):
        parts=[dict(mass_kg=1.,center=np.array([-1.,0,0]),inertia_per_kg=np.eye(3)),
               dict(mass_kg=1.,center=np.array([1.,0,0]),inertia_per_kg=np.eye(3))]
        mass,center,inertia=module.combine(parts)
        self.assertEqual(mass,2.)
        np.testing.assert_allclose(center,[0,0,0])
        np.testing.assert_allclose(inertia,np.diag([2.,4.,4.]))


if __name__=='__main__':unittest.main()
