

class IsometryWithTime
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:

    IsometryWithTime(){
      pose = Eigen::Isometry3d::Identity();
      sec = 0;
      nsec = 0;
      counter = 0;
    }

    IsometryWithTime(Eigen::Isometry3d pose_in,
                 int sec_in,
                 int nsec_in,
                 int counter_in){
      pose = pose_in;
      sec = sec_in;
      nsec = nsec_in;
      counter = counter_in;
    }

    ~IsometryWithTime(){
    }

    Eigen::Isometry3d pose;
    int sec;
    int nsec;
    int counter;
};


class PoseFileReader
{
public:

    PoseFileReader(){
    }

    ~PoseFileReader(){
    }

    void readPoseFile(std::string file_name, std::vector< IsometryWithTime > &world_to_body_poses){

        // Read the input poses file:
        ifstream in( file_name );
        vector<vector<double>> fields;
        if (in) {
            string line;
            while (getline(in, line)) {
                if (line.at(0) == '#'){
                    continue;
                }

                stringstream sep(line);
                string field;
                fields.push_back(vector<double>());
                while (getline(sep, field, ',')) {
                    fields.back().push_back(stod(field));
                }
            }
        }

        for (auto row : fields) {
            Eigen::Isometry3d world_to_body = Eigen::Isometry3d::Identity();
            world_to_body.translation() << row[3],row[4],row[5];
            world_to_body.rotate( Eigen::Quaterniond(row[9],row[6],row[7],row[8]) );

            int counter = int(row[0]);
            int sec = int(row[1]);
            int nsec = int(row[2]);
            IsometryWithTime world_to_body_pose = IsometryWithTime(world_to_body, sec, nsec, counter);
            world_to_body_poses.push_back(world_to_body_pose);
        }
    }

};