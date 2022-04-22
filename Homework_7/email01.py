
import os
from datetime import datetime
import smtplib
from smtplib import SMTP, SMTPException
import email
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

import imaplib
import time
from tkinter.messagebox import NO


def checkEmail():

    mail = imaplib.IMAP4_SSL('imap.gmail.com')
    emailadds = 'enpm809tshon@gmail.com'
    pwd = 'enpm809t'
    mail.login(emailadds, pwd)
    mail.list()  # List of folders or lables in gmail

    count = 0

    while count < 60:
        try:
            # Connect to inbox
            mail.select('inbox')

            # Search for an unread email form user's email address
            result, data = mail.search(None,
                                       f'(UNSEEN FROM "scortes3@umd.edu")')
            print(result)
            print(len(data))
            print(f'Data: {data}')

            ids = data[0]  # Data is a list
            id_list = ids.split()  # ids is a space separated by a string

            latest_email_id = id_list[-1]  # Get latest
            result, data = mail.fetch(latest_email_id, 'RFC822')

            if data is None:
                print('Waiting...')

            if data is not None:
                print('Process Initiated')
                break

        except IndexError:
            print('err')
            time.sleep(2)
            if count < 59:
                count = count + 1
                continue
            else:
                print("Gameover")
                count = 60


def main():

    # Define time stamp and record an image
    pic_time = datetime.now().strftime('%Y%m%d%H%M%S')
    cmd = f'raspistill -w 1280 -h 720 -vf -hf -o {pic_time}.jpg'
    os.system(cmd)

    # Email information
    smtpUser = 'enpm809tshon@gmail.com'
    smtpPass = 'enpm809t'

    # Destination email information
    # toAdd = 'scortes3@umd.edu'
    toAdd = ['scortes3@umd.edu', 'ENPM809TS19@gmail.com']
    fromAdd = smtpUser
    subject = f'Image recorded at {pic_time}'
    msg = MIMEMultipart()
    msg['Subject'] = subject
    msg['From'] = fromAdd
    # msg['To'] = toAdd
    msg['To'] = ', '.join(toAdd)
    msg.preamble = f'Image recorded at {pic_time}'

    # Email text
    body = MIMEText(f'Image recorded at {pic_time}')
    msg.attach(body)

    # Attach image
    fp = open(f'{pic_time}.jpg', 'rb')
    img = MIMEImage(fp.read())
    msg.attach(img)

    # Send email
    s = smtplib.SMTP('smtp.gmail.com', 587)

    s.ehlo()
    s.starttls()
    s.ehlo()

    s.login(smtpUser, smtpPass)
    s.sendmail(fromAdd, toAdd, msg.as_string())
    s.quit()

    print("Email delivered!")


if __name__ == '__main__':
    checkEmail()
    # main()
