#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <Windows.h>


using namespace std;

// decision threshold: td
#define DECISION_THRESHOLD 0.1

// 1 : With database or full search , 0: mannual search
#define RUN_TYPE_DATABASE 0

//string query_img = "9__M_Left_little_finger_Zcut.BMP";
string query_img = "1__M_Left_index_finger_Zcut.BMP";

string reference_img = "10__M_Left_index_finger.BMP";
//string reference_img = "3__M_Left_middle_finger.BMP";

// Get a list of all real files in a folder, Windows specific
vector<wstring> get_files_of_a_directory(wstring folder)
{
    vector<wstring> names;
    wstring search_path = folder + L"/*.*";
    WIN32_FIND_DATA fd;
    HANDLE hFind = ::FindFirstFile(search_path.c_str(), &fd);
    if(hFind != INVALID_HANDLE_VALUE) {
        do {
            // read all (real) files in current folder
            // without default folder . and ..
            if(! (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) ) {
                names.push_back(fd.cFileName);
            }
        }while(::FindNextFile(hFind, &fd));
        ::FindClose(hFind);
    }
    return names;
}

int main(int argc, char *argv[])
{
    cv::Mat img_1, img_2;

    img_1 = cv::imread("./Altered/Altered-Easy/" + query_img, cv::IMREAD_GRAYSCALE);

    std::string converted_str = "";
#if (RUN_TYPE_DATABASE == 1)
    std::vector <wstring> r_images = get_files_of_a_directory(L"./Real");
    for(int i = 0; i < r_images.size(); i++) {
        //apply converter (.to_bytes: wstr->str, .from_bytes: str->wstr)
        std::string converted_str( r_images[i].begin(), r_images[i].end() );
        img_2 = cv::imread("./Real/" + converted_str, cv::IMREAD_GRAYSCALE);
#else
        img_2 = cv::imread("./Real/" + reference_img, cv::IMREAD_GRAYSCALE);
#endif

        cv::imshow("Query Image: ", img_1);
        cv::imshow("Reference Image ", img_2);

        if ( !img_1.data || !img_2.data)
        {
            std::cout << "Error reading in images...." << std::endl;
            return -1;
        }

        // 1. Step: Detect keypoints using SIFT algorithm and compute the descriptors
        //
        std::vector< cv::KeyPoint > keypoints_1, keypoints_2;
        cv::Mat descriptors_1, descriptors_2;
        cv::Ptr< cv::xfeatures2d::SIFT > detector = cv::xfeatures2d::SIFT::create(); // SIFT detector
        detector->detectAndCompute( img_1, cv::Mat(), keypoints_1, descriptors_1 );
        detector->detectAndCompute( img_2, cv::Mat(), keypoints_2, descriptors_2 );

        // 2. Step: Matching the descriptors to determine point correspondences
        //
        cv::BFMatcher matcher;
        std::vector< cv::DMatch > matches;
        matcher.match(descriptors_1, descriptors_2, matches);

        // 3. Step: Find "good" matches, where: distance of match < thresholdGoodMatches
        //
        double thresholdGoodMatches = 120;
        std::vector< cv::DMatch > goodMatches;
        for (int i=0; i<matches.size(); i++)
        {
            if (matches[i].distance <= thresholdGoodMatches)
            {
                goodMatches.push_back( matches[i] );
            }
        }

        // 4. Step: Determination of matching score based on good matches and keypoints
        //
        int nbrGoodMatches = goodMatches.size();
        int p = goodMatches.size();
        int kr = keypoints_1.size();
        int kq = keypoints_2.size();
        int max_kr_kq;
        if (kr > kq) {
            max_kr_kq = kr;
        } else {
            max_kr_kq = kq;
        }
        double matching_score = (double) p / max_kr_kq;

        // 5. Step: Decision of selection based on threshold, where:
        // selected: matching score > threshold
        //

        if (matching_score >= DECISION_THRESHOLD){
            std::cout << "Query Image: " << query_img << endl;
            std::cout << "Reference Image: " << converted_str << endl;
            std::cout << "Keypoints of Query Image: " << keypoints_1.size() << std::endl;
            std::cout << "Keypoints of Reference Image: " << keypoints_2.size() << std::endl;
            std::cout << "Number of Good Matches: " << goodMatches.size() << std::endl;
            std::cout << "Matching Score: " << matching_score << endl;
            std::cout << "Matched: True"<< endl;

            // 6. Step: Draw the "good" matches
            //
            cv::Mat imgMatches;
            cv::drawMatches( img_1,
                             keypoints_1,
                             img_2,
                             keypoints_2,
                             goodMatches,
                             imgMatches,
                             cv::Scalar::all(-1),
                             cv::Scalar::all(-1),
                             std::vector<char>(),
                             cv::DrawMatchesFlags::DEFAULT);
            imshow("Good Matches", imgMatches);

        } else {
#if (RUN_TYPE_DATABASE == 0)
            std::cout << "Query Image: " << query_img << endl;
            std::cout << "Reference Image: " << reference_img << endl;
            std::cout << "Keypoints of Query Image: " << keypoints_1.size() << std::endl;
            std::cout << "Keypoints of Reference Image: " << keypoints_2.size() << std::endl;
            std::cout << "Number of Good Matches: " << goodMatches.size() << std::endl;
            std::cout << "Matching Score: " << matching_score << endl;
            std::cout << "Matched: False"<< endl;

            // 5. Step: Draw the "good" matches
            //
            cv::Mat imgMatches;
            cv::drawMatches( img_1,
                             keypoints_1,
                             img_2,
                             keypoints_2,
                             goodMatches,
                             imgMatches,
                             cv::Scalar::all(-1),
                             cv::Scalar::all(-1),
                             std::vector<char>(),
                             cv::DrawMatchesFlags::DEFAULT);
            imshow("Good Matches", imgMatches);
#endif
        }
#if (RUN_TYPE_DATABASE == 1)
    }
#endif

    cv::waitKey();

    return 0;
}
