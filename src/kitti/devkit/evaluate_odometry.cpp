// Run: rosrun aicp aicp-kitti-evaluate -g /media/snobili/SimonaHD/logs/kitti/raw/poses/oxts_poses/
// -r /home/snobili/data/outDeepLO/out_block_3/ -e exp_b

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <limits>
#include <algorithm>
#include <iostream>
#include <string>

#include "aicp_kitti/matrix.h"

// args
#include <ConciseArgs>

using namespace std;

struct CommandLineConfig {
  string gt_path;
  string result_path;
  string exp_ids;
} cl_cfg;

// static parameter
// float lengths[] = {5,10,50,100,150,200,250,300,350,400};
float lengths[] = {100,200,300,400,500,600,700,800};
int32_t num_lengths = 8;

struct errors {
  int32_t first_frame;
  float   r_err;
  float   t_err;
  float   len;
  float   speed;
  errors (int32_t first_frame,float r_err,float t_err,float len,float speed) :
    first_frame(first_frame),r_err(r_err),t_err(t_err),len(len),speed(speed) {}
};

vector<Matrix> loadPoses(string file_name) {
  vector<Matrix> poses;
  FILE *fp = fopen(file_name.c_str(),"r");
  if (!fp)
    return poses;
  while (!feof(fp)) {
    Matrix P = Matrix::eye(4);
    if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &P.val[0][0], &P.val[0][1], &P.val[0][2], &P.val[0][3],
                   &P.val[1][0], &P.val[1][1], &P.val[1][2], &P.val[1][3],
                   &P.val[2][0], &P.val[2][1], &P.val[2][2], &P.val[2][3] )==12) {
      poses.push_back(P);
    }
  }
  fclose(fp);
  return poses;
}

vector<float> trajectoryDistances (vector<Matrix> &poses) {
  vector<float> dist;
  dist.push_back(0);
  for (int32_t i=1; i<poses.size(); i++) {
    Matrix P1 = poses[i-1];
    Matrix P2 = poses[i];
    float dx = P1.val[0][3]-P2.val[0][3];
    float dy = P1.val[1][3]-P2.val[1][3];
    float dz = P1.val[2][3]-P2.val[2][3];
    dist.push_back(dist[i-1]+sqrt(dx*dx+dy*dy+dz*dz));
  }
  return dist;
}

int32_t lastFrameFromSegmentLength(vector<float> &dist,int32_t first_frame,float len) {
  for (int32_t i=first_frame; i<dist.size(); i++)
    if (dist[i]>dist[first_frame]+len)
      return i;
  return -1;
}

inline float rotationError(Matrix &pose_error) {
  float a = pose_error.val[0][0];
  float b = pose_error.val[1][1];
  float c = pose_error.val[2][2];
  float d = 0.5*(a+b+c-1.0);
  return acos(max(min(d,1.0f),-1.0f));
}

inline float translationError(Matrix &pose_error) {
  float dx = pose_error.val[0][3];
  float dy = pose_error.val[1][3];
  float dz = pose_error.val[2][3];
  return sqrt(dx*dx+dy*dy+dz*dz);
}

vector<errors> calcSequenceErrors (vector<Matrix> &poses_gt,vector<Matrix> &poses_result) {

  // error vector
  vector<errors> err;

  // parameters
  int32_t step_size = 10; // every second
  
  // pre-compute distances (from ground truth as reference)
  vector<float> dist = trajectoryDistances(poses_gt);
 
  // for all start positions do
  for (int32_t first_frame=0; first_frame<poses_gt.size(); first_frame+=step_size) {
  
    // for all segment lengths do
    for (int32_t i=0; i<num_lengths; i++) {
    
      // current length
      float len = lengths[i];
      
      // compute last frame
      int32_t last_frame = lastFrameFromSegmentLength(dist,first_frame,len);
      
      // continue, if sequence not long enough
      if (last_frame==-1)
        continue;

      // compute rotational and translational errors
      Matrix pose_delta_gt     = Matrix::inv(poses_gt[first_frame])*poses_gt[last_frame];
      Matrix pose_delta_result = Matrix::inv(poses_result[first_frame])*poses_result[last_frame];
      Matrix pose_error        = Matrix::inv(pose_delta_result)*pose_delta_gt;
      float r_err = rotationError(pose_error);
      float t_err = translationError(pose_error);
      
      // compute speed
      float num_frames = (float)(last_frame-first_frame+1);
      float speed = len/(0.1*num_frames);
      
      // write to file
      err.push_back(errors(first_frame,r_err/len,t_err/len,len,speed));
    }
  }

  // return error vector
  return err;
}

void saveSequenceErrors (vector<errors> &err,string file_name) {

  // open file  
  FILE *fp;
  fp = fopen(file_name.c_str(),"w");
 
  // write to file
  for (vector<errors>::iterator it=err.begin(); it!=err.end(); it++)
    fprintf(fp,"%d %f %f %f %f\n",it->first_frame,it->r_err,it->t_err,it->len,it->speed);
  
  // close file
  fclose(fp);
}

void savePathPlot (vector<Matrix> &poses_gt,vector<Matrix> &poses_result,string file_name) {

  // parameters
  int32_t step_size = 3;

  // open file  
  FILE *fp = fopen(file_name.c_str(),"w");
 
  // save x/y coordinates of all frames to file
  for (int32_t i=0; i<poses_gt.size(); i+=step_size)
    fprintf(fp,"%f %f %f %f\n",poses_gt[i].val[0][3],poses_gt[i].val[1][3],
                               poses_result[i].val[0][3],poses_result[i].val[1][3]);
  
  // close file
  fclose(fp);
}

vector<int32_t> computeRoi (vector<Matrix> &poses_gt,vector<Matrix> &poses_result) {
  
  float x_min = numeric_limits<int32_t>::max();
  float x_max = numeric_limits<int32_t>::min();
  float y_min = numeric_limits<int32_t>::max();
  float y_max = numeric_limits<int32_t>::min();
  
  for (vector<Matrix>::iterator it=poses_gt.begin(); it!=poses_gt.end(); it++) {
    float x = it->val[0][3];
    float y = it->val[1][3];
    if (x<x_min) x_min = x; if (x>x_max) x_max = x;
    if (y<y_min) y_min = y; if (y>y_max) y_max = y;
  }
  
  for (vector<Matrix>::iterator it=poses_result.begin(); it!=poses_result.end(); it++) {
    float x = it->val[0][3];
    float y = it->val[1][3];
    if (x<x_min) x_min = x; if (x>x_max) x_max = x;
    if (y<y_min) y_min = y; if (y>y_max) y_max = y;
  }
  
  float dx = 1.1*(x_max-x_min);
  float dy = 1.1*(y_max-y_min);
  float mx = 0.5*(x_max+x_min);
  float my = 0.5*(y_max+y_min);
  float r  = 0.5*max(dx,dy);
  
  vector<int32_t> roi;
  roi.push_back((int32_t)(mx-r));
  roi.push_back((int32_t)(mx+r));
  roi.push_back((int32_t)(my-r));
  roi.push_back((int32_t)(my+r));
  return roi;
}

void plotPathPlot (string dir,string res_dir,vector<int32_t> &roi,vector<string> &exp_ids,int32_t idx) {

  // gnuplot file name
  char command[1024];
  char file_name[256];
  sprintf(file_name,"%02d.gp",idx);
  string full_name = dir + "/" + file_name;
  // create png + eps
  for (int32_t i=0; i<2; i++) {

    // open file  
    FILE *fp = fopen(full_name.c_str(),"w");

    // save gnuplot instructions
    if (i==0) {
      fprintf(fp,"set term png size 900,900\n");
      fprintf(fp,"set output \"%02d.png\"\n",idx);
    } else {
      fprintf(fp,"set term postscript eps enhanced color\n");
      fprintf(fp,"set output \"%02d.eps\"\n",idx);
    }

    fprintf(fp,"set title 'Sequence %02d'\n",idx);
    fprintf(fp,"set size ratio -1\n");
    fprintf(fp,"set xrange [%d:%d]\n",roi[0],roi[1]);
    fprintf(fp,"set yrange [%d:%d]\n",roi[2],roi[3]);
    fprintf(fp,"set xlabel \"x [m]\"\n");
    fprintf(fp,"set ylabel \"y [m]\"\n");

    fprintf(fp,"plot \"%02d_%s.txt\" using 1:2 lc rgb \"#FF0000\" title 'Ground Truth' w lines,",idx,exp_ids.at(0).c_str());
    for (int e = 0; e < exp_ids.size(); e++)
    {
      string exp_name = exp_ids.at(e);
      replace( exp_name.begin(), exp_name.end(), '_', ' ' );
      fprintf(fp,"\"%02d_%s.txt\" using 3:4 title '%s DeepLO ' w lines,",idx,exp_ids.at(e).c_str(),exp_name.c_str());
    }
    fprintf(fp,"\"< head -1 %02d_%s.txt\" using 1:2 lc rgb \"#000000\" pt 4 ps 1 lw 2 title 'Sequence Start' w points\n",idx,exp_ids.at(0).c_str());

    // close file
    fclose(fp);

    // run gnuplot => create png + eps
    sprintf(command,"cd %s; gnuplot %s",dir.c_str(),file_name);
    system(command);
  }

  // create pdf and crop
  sprintf(command,"cd %s; ps2pdf %02d.eps %02d_large.pdf",dir.c_str(),idx,idx);
  system(command);
  sprintf(command,"cd %s; pdfcrop %02d_large.pdf %02d.pdf",dir.c_str(),idx,idx);
  system(command);
  sprintf(command,"cd %s; rm %02d_large.pdf",dir.c_str(),idx);
  system(command);
  cout << "--------------------------------------------------------" << endl;
}

void saveErrorPlots(vector<errors> &seq_err,string plot_error_dir,char* prefix) {

  // file names
  char file_name_tl[1024]; sprintf(file_name_tl,"%s/%s_tl.txt",plot_error_dir.c_str(),prefix);
  char file_name_rl[1024]; sprintf(file_name_rl,"%s/%s_rl.txt",plot_error_dir.c_str(),prefix);
  char file_name_ts[1024]; sprintf(file_name_ts,"%s/%s_ts.txt",plot_error_dir.c_str(),prefix);
  char file_name_rs[1024]; sprintf(file_name_rs,"%s/%s_rs.txt",plot_error_dir.c_str(),prefix);

  // open files
  FILE *fp_tl = fopen(file_name_tl,"w");
  FILE *fp_rl = fopen(file_name_rl,"w");
  FILE *fp_ts = fopen(file_name_ts,"w");
  FILE *fp_rs = fopen(file_name_rs,"w");
 
  // for each segment length do
  for (int32_t i=0; i<num_lengths; i++) {

    float t_err = 0;
    float r_err = 0;
    float num   = 0;

    // for all errors do
    for (vector<errors>::iterator it=seq_err.begin(); it!=seq_err.end(); it++) {
      if (fabs(it->len-lengths[i])<1.0) {
        t_err += it->t_err;
        r_err += it->r_err;
        num++;
      }
    }
    
    // we require at least 3 values
    if (num>2.5) {
      fprintf(fp_tl,"%f %f\n",lengths[i],t_err/num);
      fprintf(fp_rl,"%f %f\n",lengths[i],r_err/num);
    }
  }
  
  // for each driving speed do (in m/s)
  for (float speed=2; speed<25; speed+=2) {

    float t_err = 0;
    float r_err = 0;
    float num   = 0;

    // for all errors do
    for (vector<errors>::iterator it=seq_err.begin(); it!=seq_err.end(); it++) {
      if (fabs(it->speed-speed)<2.0) {
        t_err += it->t_err;
        r_err += it->r_err;
        num++;
      }
    }
    
    // we require at least 3 values
    if (num>2.5) {
      fprintf(fp_ts,"%f %f\n",speed,t_err/num);
      fprintf(fp_rs,"%f %f\n",speed,r_err/num);
    }
  }
  
  // close files
  fclose(fp_tl);
  fclose(fp_rl);
  fclose(fp_ts);
  fclose(fp_rs);
}

void plotErrorPlots (string dir,vector<string> &exp_ids,char* idx) {

  char command[1024];

  // for all four error plots do
  for (int32_t i=0; i<4; i++) {
 
    // create suffix
    char suffix[16];
    switch (i) {
      case 0: sprintf(suffix,"tl"); break;
      case 1: sprintf(suffix,"rl"); break;
      case 2: sprintf(suffix,"ts"); break;
      case 3: sprintf(suffix,"rs"); break;
    }
       
    // gnuplot file name
    char file_name[1024]; char full_name[1024];
    sprintf(file_name,"%s_%s.gp",idx,suffix);
    sprintf(full_name,"%s/%s",dir.c_str(),file_name);
    
    // create png + eps
    for (int32_t j=0; j<2; j++) {

      // open file  
      FILE *fp = fopen(full_name,"w");

      // save gnuplot instructions
      if (j==0) {
        fprintf(fp,"set term png size 500,250 font \"Helvetica\" 11\n");
        fprintf(fp,"set output \"%s_%s.png\"\n",idx,suffix);
      } else {
        fprintf(fp,"set term postscript eps enhanced color\n");
        fprintf(fp,"set output \"%s_%s.eps\"\n",idx,suffix);
      }
      
      fprintf(fp,"set title 'Sequence %s'\n",idx);
      // start plot at 0
      fprintf(fp,"set size ratio 0.5\n");
      fprintf(fp,"set yrange [0:*]\n");

      // x label
      if (i<=1) fprintf(fp,"set xlabel \"Path Length [m]\"\n");
      else      fprintf(fp,"set xlabel \"Speed [km/h]\"\n");
      
      // y label
      if (i==0 || i==2) fprintf(fp,"set ylabel \"Translation Error [%%]\"\n");
      else              fprintf(fp,"set ylabel \"Rotation Error [deg/m]\"\n");
      
      // plot error curve
      fprintf(fp,"plot ");
      for (int e = 0; e < exp_ids.size(); e++)
      {
        string exp_name = exp_ids.at(e);
        replace( exp_name.begin(), exp_name.end(), '_', ' ' );
        fprintf(fp," \"%s_%s_%s.txt\" using ",idx,exp_ids.at(e).c_str(),suffix);
        switch (i) {
          case 0: fprintf(fp,"1:($2*100) title '%s' pt 4 w linespoints,",exp_name.c_str()); break;
          case 1: fprintf(fp,"1:($2*57.3) title '%s' pt 4 w linespoints,",exp_name.c_str()); break;
          case 2: fprintf(fp,"($1*3.6):($2*100) title '%s' pt 4 w linespoints,",exp_name.c_str()); break;
          case 3: fprintf(fp,"($1*3.6):($2*57.3) title '%s' pt 4 w linespoints,",exp_name.c_str()); break;
        }
      }
      fprintf(fp," \n");
      
      // close file
      fclose(fp);
      
      // run gnuplot => create png + eps
      sprintf(command,"cd %s; gnuplot %s",dir.c_str(),file_name);
      system(command);
    }
    
    // create pdf and crop
    sprintf(command,"cd %s; ps2pdf %s_%s.eps %s_%s_large.pdf",dir.c_str(),idx,suffix,idx,suffix);
    system(command);
    sprintf(command,"cd %s; pdfcrop %s_%s_large.pdf %s_%s.pdf",dir.c_str(),idx,suffix,idx,suffix);
    system(command);
    sprintf(command,"cd %s; rm %s_%s_large.pdf",dir.c_str(),idx,suffix);
    system(command);
  }
}

void saveStats (vector<vector<errors>> err,string dir) {

  float t_err = 0;
  float r_err = 0;

  float num = 0;
  for (int j=0; j<err.size(); j++)
  {
    // for all errors do => compute sum of t_err, r_err
    for (vector<errors>::iterator it=err.at(j).begin(); it!=err.at(j).end(); it++) {
      t_err += it->t_err;
      r_err += it->r_err;
      num ++;
    }
  }

  // open file  
  FILE *fp = fopen((dir + "/stats.txt").c_str(),"w");
 
  // save errors
  fprintf(fp,"%f %f\n",t_err/num,r_err/num);
  
  // close file
  fclose(fp);
}

bool eval (string gt_path, string result_path, string exp_ids) {
  vector<string> exp_ids_vector;
  stringstream ss(exp_ids);
  while( ss.good() )
  {
    string substr;
    getline( ss, substr, ',' );
    exp_ids_vector.push_back( substr );
  }

  string eval_dir;
  if(exp_ids_vector.size() > 1)
    eval_dir = result_path + "kitti_evaluate";  // will be filled with evaluations
  else
    eval_dir = result_path + exp_ids_vector.at(0) + "/kitti_evaluate";

  // ground truth and result directories
  string gt_dir         = gt_path;                         // contains ground truth
  string error_dir      = eval_dir + "/errors";
  string plot_path_dir  = eval_dir + "/plot_path";
  string plot_error_dir = eval_dir + "/plot_error";

  // create output directories
  system(("mkdir " + eval_dir).c_str());
  system(("mkdir " + error_dir).c_str());
  system(("mkdir " + plot_path_dir).c_str());
  system(("mkdir " + plot_error_dir).c_str());

  // plot status
  cout << "Ground truth directory: " << gt_dir << endl;

  // total errors
  vector<vector<errors>> total_err_vector(exp_ids_vector.size());

  // for all sequences do
  for (int32_t i=0; i<11; i++) { //only test sequence:  for (int32_t i=5; i<6; i++) {
                                //all sequences:       for (int32_t i=0; i<11; i++) {
    //if ((i == 4) || (i == 5) || (i == 6) || (i == 7) || (i == 10)) //choose test sequences
    if ((i == 5))
    {
      // input file name
      char file_name[256];
      sprintf(file_name,"%02d.txt",i);

      vector<int32_t> roi_final;

      for (int j=0; j<exp_ids_vector.size(); j++)
      {
        string result_dir = result_path + exp_ids_vector.at(j) + '/';   // contains predictions
        // plot status
        cout << "Predictions directory: " << result_dir << endl;

        // read ground truth and result poses
        vector<Matrix> poses_gt     = loadPoses(gt_dir + file_name);
        vector<Matrix> poses_result = loadPoses(result_dir + "/results/" + file_name);

        // plot status
        cout << "Processing: " << file_name << ", poses: " << poses_result.size() << "/" << poses_gt.size() << endl;
        
        bool sequence_exists = true;
        // check for errors
        if (poses_gt.size()==0 || poses_result.size()!=poses_gt.size()) {
          cout << "ERROR: Couldn't read (all) poses of: " << file_name << endl;
          sequence_exists = false;
        }

        if (sequence_exists)
        {
          // output file name
          char out_file_name[256];
          sprintf(out_file_name,"%02d_%s.txt",i,exp_ids_vector.at(j).c_str());

          // compute sequence errors
          vector<errors> seq_err = calcSequenceErrors(poses_gt,poses_result);
          saveSequenceErrors(seq_err,error_dir + "/" + out_file_name);

          // add to total errors
          // total_err.insert(total_err.end(),seq_err.begin(),seq_err.end());
          total_err_vector.at(j).insert(total_err_vector.at(j).end(),seq_err.begin(),seq_err.end());

          // for first half => plot trajectory and compute individual stats
          if (i<=10) {

            // save + plot bird's eye view trajectories
            savePathPlot(poses_gt,poses_result,plot_path_dir + "/" + out_file_name);
            vector<int32_t> roi = computeRoi(poses_gt,poses_result);
            if (roi_final.empty())
              roi_final = roi;
            // else if(roi.at(0) < roi_final.at(0))
            //   roi_final.at(0) = roi.at(0);
            // else if(roi.at(1) > roi_final.at(1))
            //   roi_final.at(1) = roi.at(1);
            // else if(roi.at(2) < roi_final.at(2))
            //   roi_final.at(2) = roi.at(2);
            // else if(roi_final.at(3) > roi.at(3))
            //   roi_final.at(3) = roi.at(3);

            // plotPathPlot(plot_path_dir,roi,i);

            // save + plot individual errors
            char prefix[16];
            sprintf(prefix,"%02d_%s",i,exp_ids_vector.at(j).c_str());
            saveErrorPlots(seq_err,plot_error_dir,prefix);
          }

          plotPathPlot(plot_path_dir,result_path,roi_final,exp_ids_vector,i);
          char prefix[16];
          sprintf(prefix,"%02d",i);
          plotErrorPlots(plot_error_dir,exp_ids_vector,prefix);
        }

        // save
        if (total_err_vector.size()>0) {
          char prefix[16];
          sprintf(prefix,"avg_%s",exp_ids_vector.at(j).c_str());
          saveErrorPlots(total_err_vector.at(j),plot_error_dir,prefix);
        }
      }
    }
  }
  
  // plot total errors + summary statistics
  if (total_err_vector.size()>0) {
    char prefix[16];
    sprintf(prefix,"avg");
    plotErrorPlots(plot_error_dir,exp_ids_vector,prefix);
    saveStats(total_err_vector,error_dir);
  }
  // // save + plot total errors + summary statistics
  // if (total_err.size()>0) {
  //   char prefix[16];
  //   sprintf(prefix,"avg");
  //   saveErrorPlots(total_err,plot_error_dir,prefix);
  //   plotErrorPlots(plot_error_dir,prefix);
  //   saveStats(total_err,error_dir);
  // }

  // success
	return true;
}

int main (int argc, char **argv) {
  cl_cfg.gt_path = "";      // /media/snobili/SimonaHD/logs/kitti/raw/poses/oxts_poses/
  cl_cfg.result_path = "";  // /home/snobili/data/outDeepLO/out_block_3/
  cl_cfg.exp_ids = "";       // exp_a or exp_a,exp_b,exp_c,...

  ConciseArgs parser(argc, argv, "");
  parser.add(cl_cfg.gt_path, "g", "gt_path", "Absolute path to ground truth directory (ends with oxts_poses)");
  parser.add(cl_cfg.result_path, "r", "result_path", "Absolute path to predicted poses directory (ends with out_block_#)");
  parser.add(cl_cfg.exp_ids, "e", "exp_ids", "Experiment names list (e.g. exp_a or exp_a,exp_b,exp_c,...)");

  parser.parse();

  // run evaluation
  bool success = eval(cl_cfg.gt_path, cl_cfg.result_path, cl_cfg.exp_ids);

  return 0;
}

